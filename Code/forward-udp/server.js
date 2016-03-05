"use strict";
const SerialPort = require("serialport").SerialPort;
const dgram = require("dgram");
const port = new SerialPort(process.argv[2], {
	  baudrate: 115200
});
const server = dgram.createSocket("udp4");

var lastip;
var lastport;

port.on("open", () => {
	console.log("serial port open! ");
});

port.on("data", (data) => {
	if (lastip && lastport) {
		server.send(data, 0, data.length, lastport, lastip);
	}
});

server.on("listening", () => {
	console.log("server listening! ");
});

server.on("message", (data, info) => {
	lastip = info.address;
	lastport = info.port;
	port.write(data);
});

server.bind(1490);
