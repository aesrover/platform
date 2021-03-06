import os
import time

from flask import Flask, render_template, send_from_directory
from flask_socketio import SocketIO, emit
from pymongo.collection import Collection

from ..movement import AutoThrustThreaded

print("\nImports successfully completed\n")


WEBSERVER_FOLDER_NAME = "motorwebserver"


def normalize_motor_power(power):
    max_ = 10
    min_ = -max_
    return int(max(min_, min(max_, power)))


class ControlServer(SocketIO):
    def __init__(self, host, port, att: AutoThrustThreaded, *args, status_mongo_col: Collection=None):
        # Dynamic Variables
        self.app = Flask(__name__, static_folder=WEBSERVER_FOLDER_NAME + "/static",
                         template_folder=WEBSERVER_FOLDER_NAME + "/templates")

        super().__init__(self.app)

        self.host = host
        self.port = port

        self.att = att

        self.lastConnectTime = None
        self.previousStatusData = []

        # Get status Collection object from inputted database name
        self.db_col = status_mongo_col

        # Routes:
        self.favicon = self.app.route('/favicon.ico')(self.favicon)
        self.index = self.app.route('/')(self.index)

        # Socket endpoints:
        self.connect = self.on('connect')(self.client_connect)
        self.set_auto_state = self.on('set_auto_state')(self.set_auto_state)
        self.req_auto_state = self.on('req_auto_state')(self.req_auto_state)
        self.input_control = self.on('input')(self.input_control)
        self.poll = self.on('poll')(self.poll)
        self.client_disconnect = self.on('disconnect')(self.client_disconnect)
        self.move_depth_ticks = self.on('move_depth_ticks')(self.move_depth_ticks)
        self.get_depth_status = self.on('get_depth_status')(self.get_depth_status)
        self.reset_depth = self.on("reset_depth")(self.reset_depth)
        self.stop_depth = self.on("stop_depth")(self.stop_depth)

    def run_server(self, **kwargs):
        # Start motor drive thread:
        self.att.start()

        try:
            super().run(self.app, self.host, self.port, **kwargs)
        finally:
            print("Close thruster object:")
            self.att.close()
            print("--EXIT--")

    def favicon(self):
        return send_from_directory(os.path.join(self.app.root_path, 'static'),
                                   'favicon.ico',
                                   mimetype='image/vnd.microsoft.icon')

    @staticmethod
    def index():
        """Serve the client-side application."""
        return render_template('index.html')

    def get_status_data(self):
        status_data = []
        for data in self.db_col.find().sort([["_id", 1]]):
            del data[u'_id']
            status_data.append(data)
        return status_data

    @staticmethod
    def client_connect():
        print("Client Connect")

    def set_auto_state(self, data):
        if self.att.ws.wpm is None:
            return
        if data['state'] == 1 and len(self.att.ws.wpm.get_wps()) > 0:
            #  GET NEW TARGET
            self.att.set_auto_state(True)
        elif data['state'] == 0:
            self.att.set_auto_state(False)

    # NOTE: When getting data from VirtualJoystick, make sure to check if it is
    # "up", "down", "left", or "right" to stop when finger is lifted off
    def input_control(self, data):
        # target_bearing = 10
        gain = 1.  # 32767/80 # Make sure this is a float
        x_value = data['x'] * gain
        y_value = data['y'] * gain

        print("[Joystic Update] Joy X: {:5.2f} | Joy Y: {:5.2f}".format( x_value , y_value ))

        self.att.set_thrust(0, y_value*2, -x_value)

        # Emit status data if collection was supplied:
        if self.db_col is not None:
            emit("status", self.get_status_data())

    def req_auto_state(self, data):
        if self.att.ws.wpm is not None:
            num_wps = len(self.att.ws.wpm.get_wps())
        else:
            num_wps = 'None'
        d = {'state': self.att.auto, 'remaining': num_wps}
        emit("auto_status", d)

    # Depth:
    def move_depth_ticks(self, data):
        """ SocketIO event catch for depth move command. """
        ticks = float(data['ticks'])
        self.att.ws.depth_m.move_rel_tick(ticks, t=True)

    def get_depth_status(self, data):
        """ Emit depth satus and current position using SocketIO, on SocketIO request. """
        emit('depth_status',
             {'is_moving': self.att.ws.depth_m.is_moving(), 'curr_pos': self.att.ws.depth_m.curr_tick})

    def reset_depth(self):
        """ SocketIO event catch for depth tick reset """
        self.att.ws.depth_m.curr_tick = 0

    def stop_depth(self):
        """ SocketIO event catch to stop depth """
        self.att.ws.depth_m.stop()

    def poll(self, data):
        self.lastConnectTime = time.time()

    @staticmethod
    def client_disconnect():
        print('disconnect')

#if __name__ == '__main__':
#    cs = ControlServer(host="0.0.0.0", port=8000)
#    cs.run_server()
