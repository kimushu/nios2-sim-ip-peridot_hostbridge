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

function loopPromise(start: number, end: number, step: number, action: (offset: number) => any, result?: any): Promise<any> {
    let next = (now: number, result: any) => {
        if (((step > 0) && (now < end)) || ((step < 0) && (now > end))) {
            return Promise.resolve(action(now)).then((result) => {
                return next(now + step, result);
            })
        }
        return Promise.resolve(result);
    }
    return next(start, result);
}

class PeridotHostBridgeModule extends Module {
    public m1: AvalonMaster;
    public s1: AvalonSlave;
    public avsirq: InterruptSender;
    protected ledMask: number = 0xf;

    private _port: string;
    private _serial: SerialPort;
    private _classid: number;
    private _timecode: number;
    private _uid_l: number;
    private _uid_h: number;
    private _bid: string;
    private _sid: string;

    private _key: number;
    private _uidval: boolean;
    private _uid_en: boolean;
    private _flash_en: boolean;
    private _msg_en: boolean;
    private _user_img: boolean;
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
    private _i2cDriveNext: number;
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
        this._key = parseInt(a.embeddedsw.CMacro.CPURESET_KEY);
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
        this._i2cDriveNext = 0x30;
        this._i2cState = -2;
        this._eepAddr = 0;
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
                this._serial.on("data", (data) => {
                    this._receivePromise = this._receivePromise.then(() => {
                        return this._receiveSerial(data)
                    });
                });
                this._serial.on("error", (error) => {
                    console.error("[peridot_hostbridge] serial error:", error);
                });
            });
        }

        return promise.then(() => {
            return Module.prototype.load.call(this, moddesc);
        });
    }

    private _receiveSerial(data: Buffer): Promise<void> {
        return data.reduce((promise, byte) => {
            return promise
            .then(() => {
                if (byte === 0x3a) {
                    // Command byte for PERIDOT
                    this._receiver = this._receiveCommand;
                    return;
                }
                if (!this._receiver) {
                    this._receiver = this._receiveStart;
                }
                return this._receiver(byte);
            });
        }, Promise.resolve());
    }

    private _receiveStart(byte: number): Promise<void> | void {
        let packet: number[];
        // console.log(`receiveStart: 0x${byte.toString(16)}`);
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
                this._receiver = this._receiveChannel;
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
                this._processPacket(Buffer.from(packet));
            }
        }
    }

    private _receiveCommand(byte: number): void {
        if ((byte & 0x01) === 0) {
            throw new Error("nCONFIG asserted by hostbridge");
        }
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
                        this._i2cDriveNext = 0x30;
                    } else if (this._i2cState === 8) {
                        if ((this._i2cByte >>> 1) === EEPROM_SLAVE_ADDR) {
                            this._i2cDriveNext = 0x10;  // ACK
                        } else {
                            this._i2cDriveNext = 0x30;  // NACK
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
                            this._i2cDriveNext = 0x10;  // ACK
                        } else {
                            this._i2cDriveNext = 0x30;  // Release
                        }
                    } else {
                        // Read (Slave -> Master)
                        let bit = (this._i2cState % 9);
                        if (bit === 8) {
                            ++this._eepAddr;
                            this._i2cDriveNext = 0x30;  // Release
                        } else {
                            this._i2cDriveNext = 0x10 | (((this._eepData[this._eepAddr] >> (7 - bit)) & 1) << 5);
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
                this._i2cDriveNext = 0x30;
            }
        }
        this._serial.write(Buffer.alloc(1,
            // AS mode, nSTATUS=H, CONF_DONE=H
            0x07 | (this._lastCommand & this._i2cDrive)
        ));
        this._lastCommand = byte;
        this._i2cDrive = this._i2cDriveNext;
        this._receiver = null;
    }

    private _receiveChannel(byte: number): void {
        this._channel = byte;
        this._receiver = null;
    }

    private _processPacket(packet: Buffer): Promise<void> {
        let addr: number;
        let size: number;
        let incl: number = 1;
        let promise: Promise<any>;
        let pre: number;
        let mid: number;
        let post: number;
        let buf: Buffer;

        console.log(`processPacket(0x${packet.readUInt8(0).toString(16)})`);
        switch (packet.readUInt8(0)) {
            case 0x00:  // Write, non-incrementing address
                incl = 0;
                /* fall through */
            case 0x04:  // Write, incrementing address
                addr = packet.readUInt32BE(4);
                size = packet.readUInt16BE(2);
                pre = (4 - (addr & 3)) & 3;
                mid = (size - pre) & ~3;
                post = size - (pre + mid);
                return Promise.resolve()
                .then(() => {
                    return loopPromise(
                        0, pre, 1,
                        (offset) => this.m1.write8(addr + incl * offset, packet.readUInt8(8 + offset))
                    );
                })
                .then(() => {
                    return loopPromise(
                        pre, pre + mid, 4,
                        (offset) => this.m1.write32(addr + incl * offset, packet.readUInt32LE(8 + offset))
                    );
                })
                .then(() => {
                    return loopPromise(
                        pre + mid, pre + mid + post, 1,
                        (offset) => this.m1.write8(addr + incl * offset, packet.readUInt8(8 + offset))
                    );
                })
                .then(() => {
                    let buf: Buffer = Buffer.allocUnsafe(4);
                    buf[0] = packet[0] ^ 0x80;
                    buf[1] = 0;
                    buf.writeUInt16BE(size, 2);
                    return this._sendPacket(0, buf);
                });
            case 0x10:  // Read, non-incrementing address
                addr = packet.readUInt32BE(4);
                size = packet.readUInt16BE(2);
                return loopPromise(
                    0, size, 4,
                    () => this.m1.read32(addr, 4),
                    new Uint32Array([0])
                )
                .then((i32: Uint32Array) => {
                    console.log(`read addr=0x${addr.toString(16)}, size=0x${size.toString(16)}, result=${i32}`);
                    return this._sendPacket(0, Buffer.from(i32.buffer, i32.byteOffset, size));
                });
            case 0x14:  // Read, incrementing address
                addr = packet.readUInt32BE(4);
                size = packet.readUInt16BE(2);
                pre = (4 - (addr & 3)) & 3;
                mid = (size - pre) & ~3;
                post = size - (pre + mid);
                buf = Buffer.allocUnsafe(size);
                return Promise.resolve()
                .then(() => {
                    let readPre = (offset: number): Promise<void> => {
                        let len = pre - offset;
                        return Promise.resolve(this.m1.read8(addr + offset, len))
                        .then((i8) => {
                            if (i8) {
                                buf.set(i8.subarray(0, len), offset);
                                offset += i8.length;
                                if (offset < pre) {
                                    return readPre(offset);
                                }
                            }
                        })
                    };
                    return readPre(0);
                })
                .then(() => {
                    let readMid = (offset: number): Promise<void> => {
                        let len = (pre + mid) - offset;
                        return Promise.resolve(this.m1.read32(addr + offset, len))
                        .then((i32) => {
                            if (i32) {
                                buf.set(
                                    new Uint8Array(i32.buffer, i32.byteOffset + offset, i32.byteLength)
                                    .subarray(0, len),
                                    offset
                                );
                                offset += i32.byteLength;
                                if (offset < (pre + mid)) {
                                    return readMid(offset);
                                }
                            }
                        })
                    };
                    return readMid(pre);
                })
                .then(() => {
                    let readPost = (offset: number): Promise<void> => {
                        let len = size - offset;
                        return Promise.resolve(this.m1.read8(addr + offset, len))
                        .then((i8) => {
                            if (i8) {
                                buf.set(i8.subarray(0, len), offset);
                                offset += i8.length;
                                if (offset < size) {
                                    return readPost(offset);
                                }
                            }
                        });
                    };
                    return readPost(pre + mid);
                })
                .then(() => {
                    return this._sendPacket(0, buf);
                });
            case 0x7f:  // No transaction
            default:    // Unsupported transaction codes
                return Promise.resolve(this._sendPacket(0, Buffer.from([0xff, 0x00, 0x00, 0x00])));
        }
    }

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
        encoded = encoded.slice(0, n);
        console.log(`packet ${encoded.length} bytes:`);
        console.log(Array.from(encoded).map((v) => `0${v.toString(16)}`.substr(-2)).join(", "));
        this._serial.write(encoded.slice(0, n));
    }

    private _readReg(offset: number): number {
        console.log(`SWI readreg: ${offset}`);
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
                    if ((this._key !== 0) || (this._key === (value >>> 16))) {
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
