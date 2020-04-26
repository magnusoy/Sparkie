# !/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""


# Importing Flask components
from flask import Flask, jsonify

# Importing util
from utils.file_handler import FileHandler

# Define Flask Server
app = Flask(__name__)

# Filehandler for reading stored objects
objects = FileHandler(
    "C:\\Users\\Petter\\Documents\\Sparkie\\resources\\remote\\objects.json")


# Endpoint to get all objects
@app.route('/', methods=["GET"])
def index():
    """Returns all found objects in file."""
    content = objects.read()
    return jsonify(content)


# Run application
if __name__ == "__main__":
    app.run(host='0.0.0.0',
            port=5000,
            debug=True,
            threaded=True)
