#!flask/bin/python3
# -*- coding: utf-8 -*-

from src import app

DEBUG = True

app.run(
        debug=DEBUG,
        host='localhost',
        port=5000,
        threaded=True
    )