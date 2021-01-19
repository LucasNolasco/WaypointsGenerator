# -*- coding: utf-8 -*- 

from flask import Flask, request
from flask_restful import Resource, Api
import logging
import json

# ------------------------------------------------------------------------------
#   Nome: Server
#      Class which implements the methods to be called when the server receives
#      a new request.
# ------------------------------------------------------------------------------
class Server(Resource):
    # ------------------------------------------------------------------------------
    #  Nome: post
    #      Method to be called when the server receives a post request.
    # 
    # 	Return:
    # 		- Returns "Ok" to indicate that the request was successfully received
    #  
    # ------------------------------------------------------------------------------
    def post(self):
        for waypoint in request.json:
            print(json.dumps(waypoint).encode("utf-8"))

        return "Ok"

    # ------------------------------------------------------------------------------
    #   Nome: execThread
    #       Method responsible for the initial configuration of the server.
    # ------------------------------------------------------------------------------
    @staticmethod
    def execThread(_port):
        app = Flask(__name__)

        log = logging.getLogger('werkzeug')
        log.disabled = True
        app.logger.disabled = True

        api = Api(app)
        api.add_resource(Server, '/waypoints')  # Requests route      

        app.run(port=_port)

if __name__ == '__main__':
    Server.execThread('5002')  # Initiates the server on the port 5002