# Sparkie API
...



### Prerequisites

Install Python 3
```bash
sudo apt update
sudo apt upgrade
sudo apt-get install python3-dev libffi-dev libssl-dev python3-pip-y
```

### Installing


Install dependencies
```bash
cd ~/sparkie/api
pip3 install -r requirements.txt
```

### Examples

To run development server :
```bash
cd ~/sparkie/api
export FLASK_APP=app.py # On Linux
set FLASK_APP=app.py # On Windows
flask run
```

Press Ctrl+C in the terminal to close the server.

The development server will now run on: http://0.0.0.0:5000/


You will now also be able to use the REST-API
```bash
curl -GET http://0.0.0.0:5000/valves
curl -GET http://0.0.0.0:5000/valves/{id}

curl -GET http://0.0.0.0:5000/manometers
curl -GET http://0.0.0.0:5000/manometers/{id}

curl -GET http://0.0.0.0:5000/fire_extinguishers
curl -GET http://0.0.0.0:5000/fire_extinguishers/{id}

curl -GET http://0.0.0.0:5000/exit_signs
curl -GET http://0.0.0.0:5000/exit_signs/{id}
```

You will be able to POST, PUT and DELETE only by a valid token.
You can recieve a valid token by login

```bash
curl --user sparkie:sparkie http://localhost:5000/login
```


Check if your token is valid
```bash
curl -GET http://0.0.0.0:5000/protected?token={yourtoken}

{"message":"This is only available for people with valid tokens."}
```


```bash
...
```


## Built With

* [Python](https://www.python.org/) - Python


## Author

* **Magnus Kvendseth Øye** - [magnusoy](https://github.com/magnusoy)


## License

This project is licensed under the MIT License - see the LICENSE.md file for details


## Libraries

[Flask](http://flask.pocoo.org/)

[Flask-Marshmellow](https://flask-marshmallow.readthedocs.io/en/latest/)

[Flask-WTF](https://flask-wtf.readthedocs.io/en/stable/)

[Flask-SQLAlchemy](http://flask-sqlalchemy.pocoo.org/2.3/)
