#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from src import app
from . import db


class Valve(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    img = db.Column(db.Text(), nullable=False)
    tag = db.Column(db.String(80), nullable=False)
    is_open = db.Column(db.Boolean, nullable=False)
    normal_condition = db.Column(db.Boolean, nullable=False)
    warning = db.Column(db.Boolean, nullable=False)

    def __init__(self, img, tag, is_open, normal_condition, warning):
        self.img = img
        self.tag = tag
        self.is_open = is_open
        self.normal_condition = normal_condition
        self.warning = warning


class Manometer(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    img = db.Column(db.Text(), nullable=False)
    tag = db.Column(db.String(80), nullable=False)
    value = db.Column(db.Float, nullable=False)
    low_warning_limit = db.Column(db.Float, nullable=False)
    low_alarm_limit = db.Column(db.Float, nullable=False)
    high_warning_limit = db.Column(db.Float, nullable=False)
    high_alarm_limit = db.Column(db.Float, nullable=False)

    def __init__(self, img, tag, value, low_warning_limit, low_alarm_limit, high_warning_limit, high_alarm_limit):
        self.img = img
        self.tag = tag
        self.value = value
        self.low_warning_limit = low_warning_limit
        self.low_alarm_limit = low_alarm_limit
        self.high_warning_limit = high_warning_limit
        self.high_alarm_limit = high_alarm_limit


class FireExtinguisher(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    img = db.Column(db.Text(), nullable=False)
    on_place = db.Column(db.Boolean, nullable=False)

    def __init__(self, img, on_place):
        self.img = img
        self.on_place = on_place


class ExitSign(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    img = db.Column(db.Text(), nullable=False)
    on_place = db.Column(db.Boolean, nullable=False)

    def __init__(self, img, on_place):
        self.img = img
        self.on_place = on_place

