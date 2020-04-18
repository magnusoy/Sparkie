#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from src import app
from flask import request, jsonify, make_response
from src.models import Valve, Manometer, FireExtinguisher, ExitSign
from src import ma
from src import db
from functools import wraps
import os.path
import json
import datetime
import jwt


class ValvesSchema(ma.Schema):
    class Meta:
        # Fields to expose
        fields = ('id', 'tag', 'is_open', 'normal_condition', 'warning')

class ManometersSchema(ma.Schema):
    class Meta:
        # Fields to expose
        fields = ('id', 'tag', 'value', 'low_warning_limit', 'low_alarm_limit', 'high_warning_limit', 'high_alarm_limit')

class FireExtinguishersSchema(ma.Schema):
    class Meta:
        # Fields to expose
        fields = ('id', 'on_place')

class ExitSignsSchema(ma.Schema):
    class Meta:
        # Fields to expose
        fields = ('id', 'on_place')


valve_schema = ValvesSchema()
valves_schema = ValvesSchema(many=True)

manometer_schema = ManometersSchema()
manometers_schema = ManometersSchema(many=True)

fire_extinguisher_schema = FireExtinguishersSchema()
fire_extinguishers_schema = FireExtinguishersSchema(many=True)

exit_sign_schema = ExitSignsSchema()
exit_signs_schema = ExitSignsSchema(many=True)


if not os.path.isfile("../instance/crud.sqlite"):
    db.create_all()


def token_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        token = request.args.get('token')
        if not token:
            return jsonify({'message': 'Token is missing!'}), 403
        try:
            data = jwt.decode(token, app.config['SECRET_KEY'])
        except:
            return jsonify({'message': 'Token is invalid!'}), 403
        return f(*args, **kwargs)
    return decorated


@app.route('/unprotected')
def unprotected():
    return jsonify({'message': 'Anyone can view this!'})


@app.route('/protected')
@token_required
def protected():
    return jsonify({'message': 'This is only available for people with valid tokens.'})


@app.route('/login')
def login():
    auth = request.authorization
    if auth and auth.password == 'sparkie':
        token = jwt.encode({'user': auth.username, 'exp': datetime.datetime.utcnow(
        ) + datetime.timedelta(days=30)}, app.config['SECRET_KEY'])
        return jsonify({'token': token.decode('UTF-8')})
    return make_response('Could not verify!', 401, {'WWW-Authenticate': 'Basic realm="Login Required"'})


""" Valve API """

# endpoint to create new valve
@app.route("/valves", methods=["POST"])
@token_required
def add_valve():
    tag = request.json['tag']
    is_open = request.json['is_open']
    normal_condition = request.json['normal_condition']
    warning = request.json['warning']

    new_valve = Valve(tag, is_open, normal_condition, warning)
    db.session.add(new_valve)
    db.session.commit()
    return valve_schema.jsonify(new_valve)


# endpoint to show all valves
@app.route("/valves", methods=["GET"])
def get_valves():
    all_valves = Valve.query.all()
    result = valves_schema.dump(all_valves)
    return jsonify(result)


# endpoint to get valve detail by id
@app.route("/valves/<id>", methods=["GET"])
def detail_valve(id):
    valve = Valve.query.get(id)
    return valve_schema.jsonify(valve)


# endpoint to update valve
@app.route("/valves/<id>", methods=["PUT"])
@token_required
def update_valve(id):
    valve = Valve.query.get(id)
    tag = request.json['tag']
    is_open = request.json['is_open']
    normal_condition = request.json['normal_condition']
    warning = request.json['warning']

    valve.tag = tag
    valve.is_open = is_open
    valve.normal_condition = normal_condition
    valve.warning = warning
    
    db.session.commit()
    return valve_schema.jsonify(valve)


# endpoint to delete valve
@app.route("/valves/<id>", methods=["DELETE"])
@token_required
def delete_valve(id):
    valve = Valve.query.get(id)
    db.session.delete(valve)
    db.session.commit()

    return valves_schema.jsonify(valve)


""" Manometer API """

# endpoint to create new manometer
@app.route("/manometers", methods=["POST"])
@token_required
def add_manometer():
    tag = request.json['tag']
    value = request.json['value']
    low_warning_limit = request.json['low_warning_limit']
    low_alarm_limit = request.json['low_alarm_limit']
    high_warning_limit = request.json['high_warning_limit']
    high_alarm_limit = request.json['high_alarm_limit']

    new_manometer = Manometer(tag, value, low_warning_limit, low_alarm_limit, high_warning_limit, high_alarm_limit)
    db.session.add(new_manometer)
    db.session.commit()
    return valve_schema.jsonify(new_manometer)


# endpoint to show all manometers
@app.route("/manometers", methods=["GET"])
def get_manometers():
    all_manometers = Manometer.query.all()
    result = manometers_schema.dump(all_manometers)
    return jsonify(result)


# endpoint to get manometer detail by id
@app.route("/manometers/<id>", methods=["GET"])
def detail_manometer(id):
    manometer = Manometer.query.get(id)
    return manometer_schema.jsonify(manometer)


# endpoint to update manometer
@app.route("/manometers/<id>", methods=["PUT"])
@token_required
def update_manometer(id):
    manometer = Manometer.query.get(id)
    tag = request.json['tag']
    value = request.json['value']
    low_warning_limit = request.json['low_warning_limit']
    low_alarm_limit = request.json['low_alarm_limit']
    high_warning_limit = request.json['high_warning_limit']
    high_alarm_limit = request.json['high_alarm_limit']

    manometer.tag = tag
    manometer.value = value
    manometer.low_warning_limit = low_warning_limit
    manometer.low_alarm_limit = low_alarm_limit
    manometer.high_warning_limit = high_warning_limit
    manometer.high_alarm_limit = high_alarm_limit
    
    db.session.commit()
    return manometer_schema.jsonify(manometer)


# endpoint to delete manometer
@app.route("/manometers/<id>", methods=["DELETE"])
@token_required
def delete_manometer(id):
    manometer = Manometer.query.get(id)
    db.session.delete(manometer)
    db.session.commit()

    return manometers_schema.jsonify(manometer)


""" FireExtinguisher API """

# endpoint to create new fire extinguisher
@app.route("/fire_extinguishers", methods=["POST"])
@token_required
def add_fire_extinguisher():
    on_place = request.json['on_place']

    new_fire_extinguisher = FireExtinguisher(on_place)
    db.session.add(new_fire_extinguisher)
    db.session.commit()
    return fire_extinguisher_schema.jsonify(new_fire_extinguisher)


# endpoint to show all fire extinguishers
@app.route("/fire_extinguishers", methods=["GET"])
def get_fire_extinguishers():
    all_fire_extinguishers = FireExtinguisher.query.all()
    result = fire_extinguishers_schema.dump(all_fire_extinguishers)
    return jsonify(result)


# endpoint to get fire extinguisher detail by id
@app.route("/fire_extinguishers/<id>", methods=["GET"])
def detail_fire_extinguisher(id):
    fire_extinguisher = FireExtinguisher.query.get(id)
    return valve_schema.jsonify(fire_extinguisher)


# endpoint to update fire extinguisher
@app.route("/fire_extinguishers/<id>", methods=["PUT"])
@token_required
def update_fire_extinguisher(id):
    fire_extinguisher = FireExtinguisher.query.get(id)
    on_place = request.json['on_place']

    fire_extinguisher.tag = on_place
    db.session.commit()
    return fire_extinguisher_schema.jsonify(fire_extinguisher)


# endpoint to delete fire extinguisher
@app.route("/fire_extinguishers/<id>", methods=["DELETE"])
@token_required
def delete_fire_extinguisher(id):
    fire_extinguisher = FireExtinguisher.query.get(id)
    db.session.delete(fire_extinguisher)
    db.session.commit()

    return fire_extinguishers_schema.jsonify(fire_extinguisher)


""" ExitSign API """

# endpoint to create new exit sign
@app.route("/exit_signs", methods=["POST"])
@token_required
def add_exit_sign():
    on_place = request.json['on_place']

    new_exit_sign = ExitSign(on_place)
    db.session.add(new_exit_sign)
    db.session.commit()
    return exit_sign_schema.jsonify(new_exit_sign)


# endpoint to show all exit signs
@app.route("/exit_signs", methods=["GET"])
def get_exit_signs():
    all_exit_signs = ExitSign.query.all()
    result = exit_signs_schema.dump(all_exit_signs)
    return jsonify(result)


# endpoint to get exit sign detail by id
@app.route("/exit_signs/<id>", methods=["GET"])
def detail_exit_sign(id):
    exit_sign = ExitSign.query.get(id)
    return valve_schema.jsonify(exit_sign)


# endpoint to update exit sign
@app.route("/exit_signs/<id>", methods=["PUT"])
@token_required
def update_exit_sign(id):
    exit_sign = ExitSign.query.get(id)
    on_place = request.json['on_place']

    exit_sign.tag = on_place
    db.session.commit()
    return exit_sign_schema.jsonify(exit_sign)


# endpoint to delete exit sign
@app.route("/exit_signs/<id>", methods=["DELETE"])
@token_required
def delete_exit_sign(id):
    exit_sign = ExitSign.query.get(id)
    db.session.delete(exit_sign)
    db.session.commit()

    return exit_signs_schema.jsonify(exit_sign)
