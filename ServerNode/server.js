/*
 * z80HardEmu Server and Controller
 * Ragul Balaji & Jie Feng 2019
 */

// Web Sockets Block
const io = require('socket.io')
const server = io.listen(3000)

server.on('connection', function (socket) {
  console.log(socket.conn.id, 'Connected', socket.conn.remoteAddress)
  socket.emit('welcome', 'welcome man')
})


// Serial Handling Block
const SerialPort = require('serialport')
let EmulatorSerial

try {
  EmulatorSerial = new SerialPort('/dev/cu.usbmodem1411' /* '/dev/ttyACM0' */, { baudRate: 115200 })
  EmulatorSerial.on('data', function (data) {
    console.log(data)
  })
} catch (error) {
  console.error('☹️ Emulator Controller Serial Connection Fail')
}