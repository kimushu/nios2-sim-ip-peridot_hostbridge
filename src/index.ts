import { Module } from "nios2-sim";
import { SimulatorOptions } from "nios2-sim/types/src/simulator";
import { Qsys } from "nios2-sim/types/src/qsys";
import { SopcInfoModule } from "nios2-sim/types/src/sopcinfo";
import { AvalonMaster, AvalonSlave, InterruptSender } from "nios2-sim/types/src/interface";
import * as SerialPort from "serialport";

const EEPROM_SLAVE_ADDR = 0b1010000;

export function getModuleConstructor(kind: string) {
    return PeridotHostBridgeModule;
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
    private _receiver: (byte: number) => void;
    private _escaped: boolean;
    private _lastCommand: number;
    private _i2cByte: number;
    private _i2cDrive: number;
    private _i2cDriveNext: number;
    private _i2cState: number;
    private _i2cWrite: boolean;
    private _eepAddr: number;
    private _eepData: Uint8Array;

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
            let uid: string = program.swiUid;
            this._uid_l = parseInt(uid.substr(-8), 16);
            this._uid_h = parseInt(`0${uid.substr(0, uid.length - 8)}`, 16);
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

        if (this._port) {
            promise = new Promise<void>((resolve, reject) => {
                this._serial = new SerialPort(this._port, { baudRate: 115200 }, (err) => {
                    if (err) {
                        return reject(err);
                    }
                    console.log(`[peridot_hostbridge] Using port: ${this._port}`);
                    resolve();
                });
                this._serial.on("data", this._receiveSerial.bind(this));
                this._serial.on("error", (error) => {
                    console.error("[peridot_hostbridge] serial error:", error);
                });
            });
        }

        return promise.then(() => {
            return Module.prototype.load.call(this, moddesc);
        });
    }

    private _receiveSerial(data: Buffer): void {
        for (let byte of data) {
            if (byte === 0x3d) {
                this._escaped = true;
                continue;
            }
            if (this._escaped) {
                byte ^= 0x20;
                this._escaped = false;
            }
            if (!this._receiver) {
                this._receiver = this._receiveStart;
            }
            this._receiver(byte);
        }
    }

    private _receiveStart(byte: number): void {
        if (byte === 0x3a) {
            // Command byte for PERIDOT
            this._receiver = this._receiveCommand;
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
