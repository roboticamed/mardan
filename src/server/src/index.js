// https://joycehong0524.medium.com/simple-android-chatting-app-using-socket-io-all-source-code-provided-7b06bc7b5aff
// https://medium.com/@raj_36650/integrate-socket-io-with-node-js-express-2292ca13d891
const SERVER_PORT = 5170

const { createServer } = require("http");
const { Server } = require("socket.io");
const express = require('express')
const app = express()
const events = require('events')
const path = require('path');
const eventEmitter = new events.EventEmitter()
const httpServer = createServer(app);
const io = new Server(httpServer, {
    cors: {
        origin:'*'
    }
});

app.use(express.static(path.join(__dirname, 'Joistick')));

app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, 'Joistick', 'index.html'));
});

io.on('connection', (socket) => {
  console.log('A user connected')

  socket.on('disconnect', () => {
    console.log('A user disconnected')
  })

  socket.on('robot-command', (data) => {
    // console.log('robot-command triggered')

    const messageData = JSON.parse(data)
    eventEmitter.emit('joystickData', messageData)
    // console.log(messageData)
  })
})

const message = {
  velocityEncoder: 0.0,
  direction: 0.0,
  input: 0.0
}

// ROS-NODEJS
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
    console.log(twistMessage)
    pub.publish(twistMessage)
  })
}

httpServer.listen(SERVER_PORT,()=>{
  console.log(`listening on *:${SERVER_PORT}`)
});