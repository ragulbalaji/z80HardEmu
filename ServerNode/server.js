/*
 * z80HardEmu Server and Controller
 * Ragul Balaji & Jie Feng 2019
 */

// Emulation Block
let CpuMemory = new Array(0x10000)
CpuMemory.fill(0x0)

const ASM = require("asm80")
// Web Sockets Block
const io = require('socket.io')
const server = io.listen(3000)

server.on('connection', function (socket) {
  console.log(socket.conn.id, 'Connected', socket.conn.remoteAddress)
  socket.emit('welcome', 'welcome man')
})


// Serial Handling Block
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
let EmulatorSerial, EmulatorRawSerial

var rom = [
  //0x01,0x00,0x01,0xed,0x43,0x00,0x01,0x03,0xc3,0x03,0x00

  //0x01, 0xff, 0xff, 0xc5, 0xc3, 0x06, 0x00, 0x00, 0x00
  //0xC5, 0xC5
  //0x31, 0x00, 0x01, // LD SP, 0x0100
  //0x01, 0x00, 0x00, // LD BC,0x0
  //0x21, 0x00, 0x10, // LD HL, 0x1000
  // LOOP:
  //0xC5, // PUSH BC
  //0x03, // INC BC
  //0x7E, // LD   A,(HL)
  //0x3C, // INC   A
  //0x77, // LD   (HL),A
  //0xC3, 0x00, 0x00 // JP LOOP
  0x00,0x00, 0xc5, 0xC3, 0x00, 0x00
]
//rom = ["2".charCodeAt(0), "A".charCodeAt(0), "B".charCodeAt(0)]
for (var i = 0; i < rom.length; i++) {
  CpuMemory[i] = rom[i]
}
try {
  //EmulatorRawSerial = new SerialPort('/dev/ttyACM0', { baudRate: 115200 })
  EmulatorRawSerial = new SerialPort('/dev/tty.usbmodem14101', { baudRate: 115200 })
  EmulatorSerial = EmulatorRawSerial.pipe(new Readline({ delimiter: '\n' }))

  EmulatorSerial.on('data', function (data) {
    // data = data.toString()
    //console.log(data);
    cmds = data.split(',')

    addr = parseInt(cmds[1], 16)
    switch (cmds[0]) {
      case 'R':
        d0 = CpuMemory[addr]
        console.log('Read', '0x' + addr.toString(16), ':', '0x' + d0.toString(16), 'reply W' + String.fromCharCode(d0))
        //setTimeout(function() {
          EmulatorRawSerial.write(Buffer(['W'.charCodeAt(0), d0]))
        //}, 1000)
        break
      case 'W':
        d0 = parseInt(cmds[2], 16)
        CpuMemory[addr] = d0
        console.log('Write', '0x' + addr.toString(16), ':', '0x' + d0.toString(16))
        // console.log('W', addr.toString(16), CpuMemory[addr].toString(16), d0)

        packet = {
          func: "memory",
          memory: CpuMemory
        }
        server.emit("update",packet);
        break
      case 'S':
        cmds.shift();
        states = cmds.map(num => parseInt(num, 10))

        packet = {
          func: "states",
          states: states
        }
        server.emit("update",packet);
        break
      default:
        console.error('❌ Resetting CPU :', data)
        reset()
        break
    }
  })
} catch (error) {
  console.error('☹️ Emulator Controller Serial Connection Fail', error)
}

reset()
function reset(){ EmulatorSerial.write('R') }