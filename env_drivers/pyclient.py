#!/usr/bin/env python
'''
Created on Apr 4, 2012

@author: lanquarden
'''
import sys
import argparse
import socket
import driver
import random

if __name__ == '__main__':
    pass

# Configure the argument parser
parser = argparse.ArgumentParser(description='Python client to connect to the TORCS SCRC server.')

parser.add_argument('--host', action='store', dest='host_ip', default='localhost',
                    help='Host IP address (default: localhost)')
parser.add_argument('--port', action='store', type=int, dest='host_port', default=3001,
                    help='Host port number (default: 3001)')
parser.add_argument('--id', action='store', dest='id', default='SCR',
                    help='Bot ID (default: SCR)')
parser.add_argument('--maxEpisodes', action='store', dest='max_episodes', type=int, default=1,
                    help='Maximum number of learning episodes (default: 1)')
parser.add_argument('--maxSteps', action='store', dest='max_steps', type=int, default=0,
                    help='Maximum number of steps (default: 0)')
parser.add_argument('--track', action='store', dest='track', default=None,
                    help='Name of the track')
parser.add_argument('--stage', action='store', dest='stage', type=int, default=3,
                    help='Stage (0 - Warm-Up, 1 - Qualifying, 2 - Race, 3 - Unknown)')
parser.add_argument('--direction', action='store', dest='direction', default='FORWARD',
                    help='direction: FORWARD or OPPOSITE (default: FORWARD)')
parser.add_argument('--lane', action='store', dest='lane', type=int, default=0,
                    help='lane - lane to keep - 0 is the rightmost, numberOfLanes the leftmost. This is with respect to agent direction (FORWARD or OPPOSITE) (defualt: 0)')
parser.add_argument('--baseSpeed', action='store', dest='baseSpeed', type=int, default=120,
                    help='baseSpeed - the mean speed of the agent (stochastic). (Default: 120)')
parser.add_argument('--stopProbability', action='store', dest='stopProbability', type=float, default=0.0,
                    help='stopProbability - the probability the agent will halt (default: 0.0)')
parser.add_argument('--switchLaneProbability', action='store', dest='switchLaneProbability', type=float, default=0.0,
                    help='switchLaneProbability - the probability  the agent will switch lanes (default: 0.0)')
parser.add_argument('--lanes', action='store', dest='lanes', default='FORWARD,FORWARD',
                    help='lanes - the directions of all the lanes (FORWARD or OPPOSITE for each lane, with a comma between) (default: FORWARD,FORWARD) - left is rightmost lane')
parser.add_argument('--seed', action='store', dest='seed', default=random.randint(0, 100),
                    help='seed - for the random behaviour. if not specified will still have some randomness')
arguments = parser.parse_args()

# Print summary
print 'Connecting to server host ip:', arguments.host_ip, '@ port:', arguments.host_port
print 'Bot ID:', arguments.id
print 'Maximum episodes:', arguments.max_episodes
print 'Maximum steps:', arguments.max_steps
print 'Track:', arguments.track
print 'Stage:', arguments.stage
print '*********************************************'

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error, msg:
    print 'Could not make a socket.'
    sys.exit(-1)

# one second timeout
sock.settimeout(1.0)

shutdownClient = False
curEpisode = 0

verbose = False

d = driver.Driver(arguments.stage, arguments.lane, arguments.lanes, False, True, arguments.seed)

while not shutdownClient:
    while True:
        print 'Sending id to server: ', arguments.id
        buf = arguments.id + d.init()
        print 'Sending init string to server:', buf

        try:
            sock.sendto(buf, (arguments.host_ip, arguments.host_port))
        except socket.error, msg:
            print "Failed to send data...Exiting..."
            sys.exit(-1)

        try:
            buf, addr = sock.recvfrom(1000)
        except socket.error, msg:
            print "didn't get response from server..."

        if buf.find('***identified***') >= 0:
            print 'Received: ', buf
            break

    currentStep = 0

    while True:
        # wait for an answer from server
        buf = None
        try:
            buf, addr = sock.recvfrom(1000)
        except socket.error, msg:
            print "didn't get response from server..."

        if verbose:
            print 'Received: ', buf

        if buf != None and buf.find('***shutdown***') >= 0:
            d.onShutDown()
            shutdownClient = True
            print 'Client Shutdown'
            break

        if buf != None and buf.find('***restart***') >= 0:
            d.onRestart()
            print 'Client Restart'
            break

        currentStep += 1
        if currentStep != arguments.max_steps:
            if buf != None:
                buf = d.drive(buf, arguments.direction,
                              arguments.baseSpeed, arguments.stopProbability, arguments.switchLaneProbability)
        else:
            buf = '(meta 1)'

        if verbose:
            print 'Sending: ', buf

        if buf != None:
            try:
                sock.sendto(buf, (arguments.host_ip, arguments.host_port))
            except socket.error, msg:
                print "Failed to send data...Exiting..."
                sys.exit(-1)

    curEpisode += 1

    if curEpisode == arguments.max_episodes:
        shutdownClient = True

sock.close()