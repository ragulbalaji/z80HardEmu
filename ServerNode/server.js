/*
 * z80HardEmu Server and Controller
 * Ragul Balaji & Jie Feng 2019
 */

// Web Sockets Block
const io = require('socket.io')
const server = io.listen(3000)

let CpuMemory = new Array(0x10000);
CpuMemory.fill(0);
server.on('connection', function (socket) {
  console.log(socket.conn.id, 'Connected', socket.conn.remoteAddress)
  socket.emit('welcome', 'welcome man')
})


// Serial Handling Block
const SerialPort = require('serialport')
let EmulatorSerial

var rom = [
  //0x01, 0xff, 0xff, 0xc5, 0xc3, 0x06, 0x00, 0x00, 0x00
  0x31, 0x00, 0x0F, //LD SP, 0x0F00
  0x01, 0x00, 0x00, //LD   BC,0x0
  0x21, 0x00, 0x10, //LD HL, 0x1000
//LOOP:
  0xC5, //PUSH BC
  0x03, //INC BC
  0x7E, //LD   A,(HL)
  0x3C, //INC   A
  0x77, //LD   (HL),A
  0xC3, 0x06, 0x00 //JP LOOP
]

for(var i = 0;i<rom.length;i++)
{
  CpuMemory[i] = rom[i];
}
try {
  EmulatorSerial = new SerialPort(/*dev/cu.usbmodem1411'*/ '/dev/ttyACM0', { baudRate: 115200 })

  EmulatorSerial.on('data', function (data) {
    console.log(data.toString());
    cmds = data.toString().split(",");
    addr = parseInt(cmds[1],16);
    switch(cmds[0])
    {
      case 'R':
        EmulatorSerial.write(new Buffer(["W".charCodeAt(0), CpuMemory[addr]]));
        break;
      case 'W':
        data = parseInt(cmds[2],16);
        CpuMemory[addr] = data;
        console.log(addr.toString(16), CpuMemory[addr].toString(16), data);
        break;
    }

  })
} catch (error) {
  console.error('☹️ Emulator Controller Serial Connection Fail')
}

EmulatorSerial.write("A");
