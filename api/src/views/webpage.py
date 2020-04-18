#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import request, render_template, flash
from src import app


@app.route("/")
@app.route('/home')
@app.route('/index')
def home():
    return render_template("index.html")


