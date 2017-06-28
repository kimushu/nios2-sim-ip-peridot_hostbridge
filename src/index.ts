import { Module } from "nios2-sim";
import { SimulatorOptions } from "nios2-sim/types/src/simulator";
import { Qsys } from "nios2-sim/types/src/qsys";
import { SopcInfoModule } from "nios2-sim/types/src/sopcinfo";
import { AvalonMaster, AvalonSlave, InterruptSender } from "nios2-sim/types/src/interface";
import * as SerialPort from "serialport";
import { then } from "nios2-sim/types/src/promiseable";

const EEPROM_SLAVE_ADDR = 0b1010000;

export function getModuleConstructor(kind: string) {
    return PeridotHostBridgeModule;
}

class PeridotHostBridgeModule extends Module {
    /* Interfaces */
    public m1: AvalonMaster;
    public s1: AvalonSlave;
    public avsirq: InterruptSender;

    /** LED mask */
    protected ledMask: number = 0xf;

    /** Port name (COMxx, /dev/xxx) */
    private _port: string;
    /** Serial port instance */
    private _serial: SerialPort;

    /** Class ID (from sopcinfo) */
    private _classid: number;
    /** Timecode (from sopcinfo) */
    private _timecode: number;
    /** Unique ID Low word (from simulator option) */
    private _uid_l: number;
    /** Unique ID High word (from simulator option) */
    private _uid_h: number;
    /** Board ID (from simulator option) */
    private _bid: string;
    /** Serial ID (from simulator option) */
    private _sid: string;

    /** Reset key */
    private _resetKey: number;
    /** UID valid */
    private _uidval: boolean;
    /** UID feature enabled */
    private _uid_en: boolean;
    /** Flash feature enabled */
    private _flash_en: boolean;
    /** Message/SWI feature enabled */
    private _msg_en: boolean;
    /** User configuration image enabled */
    private _user_img: boolean;
    /** LED status */
    private _led: number;

    private _irqena: boolean;
    private _rdy: boolean;
    private _ss: boolean;
    private _rxdata: number;

    private _message: number;
    private _swi: boolean;

    private _receivePromise: Promise<void>;
    private _receiver: (byte: number) => (Promise<void> | void);
    private _escaped3: boolean = false;

    private _lastCommand: number;
    private _i2cByte: number;
    private _i2cDrive: number;
    private _i2cState: number;
    private _i2cWrite: boolean;
    private _eepAddr: number;
    private _eepData: Uint8Array;

    private _escaped7: boolean = false;
    private _channel: number = 0;
    private _packets: Array<number[]> = [];

    constructor(path: string, system: Qsys, options: SimulatorOptions) {
        super(path, system, options);
        this._parseOptions(process.argv);
    }

    private _parseOptions(argv: string[]) {
        const program = require("commander");
        program
        .option("--swi-port <path>", "Serial port name")
        .option("--swi-bid <str>", "Board ID (J72*)")
        .option("--swi-sid <str>", "Serial ID (XXXXXX-XXXXXX-XXXXXX)")
        .option("--swi-uid <hex>", "64-bit unique ID")
        .option("--swi-image <image>", "Configuration image", /^(boot|user)$/, "user")
        .parse(argv);
        this._port = program.swiPort;
        if (program.swiUid != null) {
            let uid: string = `${"0".repeat(16)}${program.swiUid.replace(/^0x/i, "")}`.substr(-16);
            this._uid_l = parseInt(uid.substr(8), 16);
            this._uid_h = parseInt(uid.substr(0, 8), 16);
        } else {
            this._uid_l = 0;
            this._uid_h = 0;            
        }
        this._user_img = (program.swiImage === "user");
        this._bid = `${(program.swiBid || "J72X")}____}`.substr(0, 4);
        this._sid = `${(program.swiSid || "SIMULA-TDHOST-BRIDGE").replace(/-/g, "")
            }${"_".repeat(18)}`.substr(0, 18);
    }

    load(moddesc: SopcInfoModule): Promise<void> {
        let a = moddesc.assignment;
        let i = moddesc.interface;
        this._resetKey = parseInt(a.embeddedsw.CMacro.CPURESET_KEY);
        this._classid = parseInt(a.embeddedsw.CMacro.ID);
        this._timecode = parseInt(a.embeddedsw.CMacro.TIMESTAMP);
        this._flash_en = (a.embeddedsw.CMacro.USE_EPCSBOOT === "1");
        this._msg_en = (a.embeddedsw.CMacro.USE_MESSAGE === "1");
        this._uid_en = (a.embeddedsw.CMacro.USE_UIDREAD === "1");

        this._led = 0;
        this._irqena = false;
        this._rdy = false;
        this._ss = false;
        this._rxdata = 0;
        this._message = 0;
        this._swi = false;

        this._lastCommand = 0x39;
        this._i2cDrive = 0x30;
        this._i2cState = -2;
        this._eepAddr = 0;

        // Initialize EEPROM
        this._eepData = new Uint8Array(256);
        this._eepData.set([0x4a, 0x37, 0x57, 0x02], 0);
        this._eepData.set(Buffer.from(this._bid), 4);
        this._eepData.set(Buffer.from(this._sid), 8);

        this.m1 = <AvalonMaster>this.loadInterface(i.m1);
        this.s1 = <AvalonSlave>this.loadInterface(i.s1);
        this.s1.readReg = this._readReg.bind(this);
        this.s1.writeReg = this._writeReg.bind(this);
        this.avsirq = <InterruptSender>this.loadInterface(i.avsirq);

        let promise = Promise.resolve();
        this._receivePromise = Promise.resolve();

        if (this._port) {
            promise = new Promise<void>((resolve, reject) => {
                this._serial = new SerialPort(this._port, { baudRate: 115200 }, (err) => {
                    if (err) {
                        return reject(err);
                    }
                    console.log(`[peridot_hostbridge] Using port: ${this._port}`);
                    resolve();
                });
                this._serial.flush();
                this._serial.on("data", (data: Buffer) => {
                    this._receivePromise = this._receivePromise.then(() => {
                        return this._recvSerial(data)
                    });
                });
                this._serial.on("error", (error) => {
                    console.error(`[peridot_hostbridge] serial error: ${error}`);
                });
            });
        }

        return promise.then(() => {
            return Module.prototype.load.call(this, moddesc);
        });
    }

    /**
     * Process received data from serial
     * @param data Received data
     */
    private _recvSerial(data: Buffer): Promise<void> {
        return data.reduce((promise, byte) => {
            return promise
            .then(() => {
                if (byte === 0x3a) {
                    // Command byte for PERIDOT
                    this._receiver = this._recvCommand;
                    return;
                }
                if (!this._receiver) {
                    this._receiver = this._recvPacket;
                }
                return this._receiver(byte);
            });
        }, Promise.resolve());
    }

    /**
     * Process data for Avalon-ST packet
     * @param byte Byte received
     */
    private _recvPacket(byte: number): Promise<void> | void {
        let packet: number[];
        if (byte === 0x3d) {
            this._escaped3 = true;
            return;
        }
        if (this._escaped3) {
            byte ^= 0x20;
            this._escaped3 = false;
        }
        switch (byte) {
            case 0x7c:
                // Channel prefix
                this._receiver = this._recvChannel;
                return;
            case 0x7a:
                // SOP
                this._packets[this._channel] = [];
                return;
            case 0x7b:
                // EOP
                packet = this._packets[this._channel];
                if (packet) {
                    packet["eop"] = true;
                }
                return;
            case 0x7d:
                // Escape
                this._escaped7 = true;
                return;
        }
        if (this._escaped7) {
            byte ^= 0x20;
            this._escaped7 = false;
        }
        packet = this._packets[this._channel];
        if (packet) {
            packet.push(byte);
            if (packet["eop"]) {
                this._packets[this._channel] = null;
                return this._processPacket(Buffer.from(packet));
            }
        }
    }

    /**
     * Process configuration command byte
     * @param byte Byte received
     */
    private _recvCommand(byte: number): void {
        if ((byte & 0x01) === 0) {
            throw new Error("nCONFIG asserted by hostbridge");
        }
        let i2cDriveOld = this._i2cDrive;
        if ((byte & 0x80) === 0) {
            // I2C emulation
            let change = (byte ^ this._lastCommand) & 0x30;
            if (change & 0x10) {
                // clock change
                if (byte & 0x10) {
                    // rise edge (sample)
                    this._i2cByte = ((this._i2cByte << 1) | (byte & 0x20 ? 1 : 0)) & 0xff;
                } else {
                    // fall edge (change)
                    ++this._i2cState;
                    if (this._i2cState < 8) {
                        this._i2cDrive = 0x30;
                    } else if (this._i2cState === 8) {
                        if ((this._i2cByte >>> 1) === EEPROM_SLAVE_ADDR) {
                            this._i2cDrive = 0x10;  // ACK
                        } else {
                            this._i2cDrive = 0x30;  // NACK
                            this._i2cState = -2;    // Ignore transaction
                        }
                        this._i2cWrite = (this._i2cByte & 0x01) === 0;
                    } else if (this._i2cWrite) {
                        // Write (Master -> Slave)
                        if ((this._i2cState % 9) === 8) {
                            if (this._i2cState < 18) {
                                this._eepAddr = this._i2cByte;
                            } else {
                                // Write protected
                            }
                            this._i2cDrive = 0x10;  // ACK
                        } else {
                            this._i2cDrive = 0x30;  // Release
                        }
                    } else {
                        // Read (Slave -> Master)
                        let bit = (this._i2cState % 9);
                        if (bit === 8) {
                            ++this._eepAddr;
                            this._i2cDrive = 0x30;  // Release
                        } else {
                            this._i2cDrive = 0x10 | (((this._eepData[this._eepAddr] >> (7 - bit)) & 1) << 5);
                        }
                    }
                }
            } else if ((change & 0x20) && (byte & 0x10)) {
                // data change
                if (byte & 0x20) {
                    // rise edge => STOP condition
                    this._i2cState = -2;
                } else {
                    // fall edge => START condition
                    this._i2cState = -1;
                    this._i2cByte = 0;
                }
                this._i2cDrive = 0x30;
            }
        }
        this._serial.write(Buffer.alloc(1,
            // AS mode, nSTATUS=H, CONF_DONE=H
            0x07 | (this._lastCommand & i2cDriveOld)
        ));
        this._lastCommand = byte;
        this._receiver = null;
    }

    /**
     * Receive channel indicator for Avalon-ST packet
     * @param byte Byte received
     */
    private _recvChannel(byte: number): void {
        this._channel = byte;
        this._receiver = null;
    }

    /**
     * Process Avalon-ST packet
     * @param packet Packet data
     */
    private _processPacket(packet: Buffer): Promise<void> {
        let ofst: number;
        let addr: number;
        let size: number;
        let incl: boolean = true;
        let resp: Buffer;

        let type = packet.readUInt8(0);
        switch (type) {
            case 0x00:  // Write, non-incrementing address
                incl = false;
                /* fall through */
            case 0x04:  // Write, incrementing address
                addr = packet.readUInt32BE(4);
                size = packet.readUInt16BE(2);
                ofst = 8;
                resp = Buffer.from([type ^ 0x80, 0, 0, 0]);
                resp.writeUInt16BE(size, 2);
                let write = () => {
                    let p;
                    if ((size >= 4) && ((addr & 3) === 0)) {
                        p = this.m1.write32(addr, packet.readUInt32LE(ofst));
                        if (incl) { addr += 4; }
                        size -= 4;
                        ofst += 4;
                    } else {
                        p = this.m1.write8(addr, packet.readUInt8(ofst));
                        if (incl) { addr += 1; }
                        size -= 1;
                        ofst += 1;
                    }
                    return Promise.resolve(p)
                    .then(() => (size > 0) ? write() : null);
                };
                return write().then(() => this._sendPacket(0, resp));
            case 0x10:  // Read, non-incrementing address
                incl = false;
                /* fall through */
            case 0x14:  // Read, incrementing address
                addr = packet.readUInt32BE(4);
                size = packet.readUInt16BE(2);
                resp = Buffer.allocUnsafe(size);
                let read = () => {
                    let p;
                    if ((size >= 4) && ((addr & 3) === 0)) {
                        p = this.m1.read32(addr, size & ~3);
                    } else {
                        p = this.m1.read8(addr, size);
                    }
                    return Promise.resolve(p)
                    .then((i: Int32Array | Int8Array) => {
                        if (!i) {
                            if ((size >= 4) && ((addr & 3) === 0)) {
                                i = new Int32Array([0]);
                            } else {
                                i = new Int8Array([0]);
                            }
                        }
                        let len = Math.min(i.byteLength, size);
                        resp.set(Buffer.from(i.buffer, i.byteOffset, len), ofst);
                        if (incl) { addr += len; }
                        size -= len;
                        ofst += len;
                        return (size > 0) ? read() : null;
                    });
                };
                return read().then(() => this._sendPacket(0, resp));
            case 0x7f:  // No transaction
            default:    // Unsupported transaction codes
                return Promise.resolve(
                    this._sendPacket(0, Buffer.from([0xff, 0x00, 0x00, 0x00]))
                );
        }
    }

    /**
     * Send packet
     * @param channel Avalon-ST channel
     * @param data Packet content
     */
    private _sendPacket(channel: number, data: Buffer): void {
        let encoded: Buffer = Buffer.allocUnsafe(data.length * 2 + 4);
        let n = 0;
        encoded[n++] = 0x7c;
        encoded[n++] = channel;
        encoded[n++] = 0x7a;
        for (let i = 0; i < data.length; ++i) {
            if ((i + 1) === data.length) {
                encoded[n++] = 0x7b;
            }
            let byte = data[i];
            switch (byte) {
                case 0x7a:
                case 0x7b:
                case 0x7c:
                case 0x7d:
                    encoded[n++] = 0x7d;
                    encoded[n++] = byte ^ 0x20;
                    break;
                default:
                    encoded[n++] = byte;
            }
        }
        this._serial.write(encoded.slice(0, n));
    }

    /**
     * CSR read emulation
     * @param offset Register offset
     */
    private _readReg(offset: number): number {
        switch (offset) {
            case 0:
                return this._classid;
            case 1:
                return this._timecode;
            case 2:
                return this._uid_en ? this._uid_l : 0;
            case 3:
                return this._uid_en ? this._uid_h : 0;
            case 4:
                return (
                    (this._uidval   ? 0x8000 : 0) |
                    (this._uid_en   ? 0x4000 : 0) |
                    (this._flash_en ? 0x2000 : 0) |
                    (this._msg_en   ? 0x1000 : 0) |
                    (this._user_img ? 0x0800 : 0) |
                    (this._led & this.ledMask)
                );
            case 5:
                return this._flash_en ? (
                    (this._irqena   ? 0x8000 : 0) |
                    (this._rdy      ? 0x0200 : 0) |
                    (this._ss       ? 0x0100 : 0) |
                    (this._rxdata   & 0x03ff)
                ) : 0;
            case 6:
                return this._msg_en ? this._message : 0;
            case 7:
                return this._msg_en ? (
                    (this._swi ? 0x0001 : 0)
                ) : 0;
        }
    }

    /**
     * CSR write emulation
     * @param offset Register offset
     * @param value Value to write
     */
    private _writeReg(offset: number, value: number): boolean {
        switch (offset) {
            case 0:
            case 1:
            case 2:
            case 3:
                // Read only
                return true;
            case 4:
                this._led = value & this.ledMask;
                if (value & 0x0100) {
                    // Reset
                    if ((this._resetKey !== 0) || (this._resetKey === (value >>> 16))) {
                        throw new Error("Nios II reset by PERIDOT SWI");
                    } else {
                        this.options.printWarn("Nios II reset requested with incorrect key");
                    }
                }
                return true;
            case 5:
                if (this._flash_en) {
                    this._irqena = !!(value & 0x8000);
                    this._ss = !!(value & 0x0100);
                }
                return true;
            case 6:
                if (this._msg_en) {
                    this._message = value;
                }
                return true;
            case 7:
                if (this._msg_en) {
                    this._swi = !!(value & 0x0001);
                    if (this._swi) {
                        this.avsirq.assert();
                    } else {
                        this.avsirq.deassert();
                    }
                }
                return true;
        }
        return false;
    }
}
