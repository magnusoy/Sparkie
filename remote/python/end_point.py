# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script is the REST/API, that reads from the
file that stores the CNN output. Then pulls it up
where anyone can access the data.
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
