const SERVER_PORT = 5170

const app = require('express')()
const server = require('http').Server(app)
const io = require('socket.io')(server)
const events = require('events')

var eventEmitter = new events.EventEmitter()

app.get('/', (req, res) => {
  res.sendFile(__dirname + '/index.html')
});

server.listen(SERVER_PORT, () => {
  console.log(`listening on *:${SERVER_PORT}`)
})

io.on('connection', (socket) => {
  console.log('A user connected')

  socket.on('disconnect', () => {
    console.log('A user disconnected')
  })

  socket.on('robot-command', (data) => {
    const messageData = JSON.parse(data)
    eventEmitter.emit('joystickData', messageData)
  })
})

if (process.env.ROS_DISTRO == "noetic") {
  const rosnodejs = require('rosnodejs')

  rosnodejs.initNode('/my_node')
    .then(() => {
      // do stuff
    })

  const nh = rosnodejs.nh

  const twistMessage = {
    linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    }
  }

  const pub = nh.advertise('/motors/motor_twist', 'geometry_msgs/Twist')
  eventEmitter.on('joystickData', (data) => {
    twistMessage.linear.x = data.steering
    twistMessage.angular.z = data.throttle
    pub.publish(twistMessage)
  })
}
