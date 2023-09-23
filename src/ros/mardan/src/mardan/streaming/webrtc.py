
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


import threading

import numpy as np

import asyncio
import logging
import ssl
import argparse
import json
import os
import platform

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

from av import VideoFrame


__all__ = [
    'RosImageTopicVideoTrack',
    'RawVideoTrack',
    'WebRTCServer'
]

HTML_CONTENT = """
<html>
<head>
    <meta charset="UTF-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WebRTC webcam</title>
    <style>
    button {
        padding: 8px 16px;
    }

    video {
        width: 100%;
    }

    .option {
        margin-bottom: 8px;
    }

    #media {
        max-width: 1280px;
    }
    </style>
</head>
<body>

<div class="option">
    <label for="topic_name">Topic name:</label>
    <input type="text" id="topic_name" name="topic_name"><br><br>
    <input id="use-stun" type="checkbox"/>
    <label for="use-stun">Use STUN server</label>
</div>
<button id="start" onclick="start()">Start</button>
<button id="stop" style="display: none" onclick="stop()">Stop</button>

<div id="media">
    <h2>Media</h2>

    <audio id="audio" autoplay="true"></audio>
    <video id="video" autoplay="true" playsinline="true"></video>
</div>

<script src="client.js"></script>
</body>
</html>
"""

JAVASCRIPT_CODE = """
var pc = null;

function negotiate() {
    pc.addTransceiver('video', {direction: 'recvonly'});
    pc.addTransceiver('audio', {direction: 'recvonly'});
    return pc.createOffer().then(function(offer) {
        return pc.setLocalDescription(offer);
    }).then(function() {
        // wait for ICE gathering to complete
        return new Promise(function(resolve) {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(function() {
        var offer = pc.localDescription;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
                topic: document.getElementById('topic_name').value
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then(function(response) {
        return response.json();
    }).then(function(answer) {
        return pc.setRemoteDescription(answer);
    }).catch(function(e) {
        alert(e);
    });
}

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    if (document.getElementById('use-stun').checked) {
        config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    }

    pc = new RTCPeerConnection(config);

    // connect audio / video
    pc.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

function stop() {
    document.getElementById('stop').style.display = 'none';

    // close peer connection
    setTimeout(function() {
        pc.close();
    }, 500);
}
"""


class RosImageTopicVideoTrack(VideoStreamTrack):

    def __init__(self, topic, resolution):
        super().__init__()

        self.__subscriber = rospy.Subscriber(
            topic, Image, self.__topic_callback, queue_size=1)

        self.__cv_bridge = CvBridge()

        self.__img_lock = threading.Lock()
        self.__img_buffer = np.zeros(
            (resolution[0], resolution[1], 3), dtype=np.uint8)
        self.__resolution = resolution

    async def recv(self):

        pts, time_base = await self.next_timestamp()

        with self.__img_lock:
            frame = VideoFrame.from_ndarray(self.__img_buffer)

        frame.pts = pts
        frame.time_base = time_base
        return frame

    def __topic_callback(self, msg):

        img = self.__cv_bridge.imgmsg_to_cv2(msg)
        img = cv2.resize(img, (self.__resolution[1], self.__resolution[0]))
        with self.__img_lock:
            self.__img_buffer[...] = img[...]


class RawVideoTrack(VideoStreamTrack):

    def __init__(self, resolution):
        super().__init__()

        self.__img_lock = threading.Lock()
        self.__img_buffer = np.zeros(
            (resolution[0], resolution[1], 3), dtype=np.uint8)
        self.__resolution = resolution

    async def recv(self):

        pts, time_base = await self.next_timestamp()

        with self.__img_lock:
            frame = VideoFrame.from_ndarray(self.__img_buffer)

        frame.pts = pts
        frame.time_base = time_base
        return frame

    def update(self, img):

        img = cv2.resize(img, (self.__resolution[1], self.__resolution[0]))

        # TODO: This might not be the actual color space of the input image
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        with self.__img_lock:
            self.__img_buffer[...] = img[...]


class WebRTCServer(object):

    def __init__(self, app):

        # list of peer connections
        self.__pcs = set()

        # list of available tracks
        self.__tracks = list()

        self.__subscribers = dict()

        # configure the endpoints served by this class
        app.on_shutdown.append(self.__on_shutdown)
        app.router.add_get("/", self.__serve_index)
        app.router.add_get("/client.js", self.__serve_javascript)
        app.router.add_post("/offer", self.__serve_offer)

    def add_video_track(self, track):

        self.__tracks.append(track)

        for pc in self.__pcs:
            pc.addTrack(track)

    async def __serve_index(self, request):
        return web.Response(content_type='text/html', text=HTML_CONTENT)

    async def __serve_javascript(self, request):
        return web.Response(content_type='application/javascript', text=JAVASCRIPT_CODE)

    async def __on_shutdown(self, app):

        # close peer connections
        coros = [pc.close() for pc in self.__pcs]
        await asyncio.gather(*coros)
        self.__pcs.clear()

    async def __serve_offer(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self.__pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("Connection state is %s" % pc.connectionState)
            if pc.connectionState == "failed":
                await pc.close()
                self.__pcs.discard(pc)

        # topic the user is willing to subscribe to
        topic = params["topic"]
        print("topic: {0}".format(topic))

        if len(topic) > 0:

            subscriber = None

            # check if there is a subscriber for the given topic
            if topic in self.__subscribers.keys():
                subscriber = self.__subscribers[topic]
            else:
                print("Creating new subscriber for topic {0}".format(topic))

                # create a new subscriber
                subscriber = RosImageTopicVideoTrack(topic, (240, 320))

            self.__subscribers[topic] = subscriber
            pc.addTrack(subscriber)

        else:
            # offer all currently available tracks
            for track in self.__tracks:
                pc.addTrack(track)

        await pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )
