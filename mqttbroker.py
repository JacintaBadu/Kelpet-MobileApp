import os
import django
import json
import paho.mqtt.client as mqtt

from django.utils import timezone

from django.conf import settings
from google.cloud.firestore_v1 import FieldFilter 

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from datetime import datetime

cred = credentials.Certificate(r"C:/Users/Jacinta Badu/OneDrive/Desktop/BackendCode/BackendCode/PetApp/kelpet-admin-indicator.json")
firebase_admin.initialize_app(cred)

db = firestore.client()


# Set the default settings module for Django
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'PetApp.settings')

# Setup Django
django.setup()

# Now import the models after Django is setup
from ketlit.models import TemperatureReading, HeartRateReading, GPSReading, ActivityLevel, CollarBattery, Collar,HeartRateVariability

# MQTT Settings
MQTT_SERVER = settings.MQTT_SERVER
MQTT_PORT = settings.MQTT_PORT
MQTT_USER = settings.MQTT_USER
MQTT_PASSWORD = settings.MQTT_PASSWORD

# Topics
temperature_topic = "pet_tracker/temperature"
heartrate_topic = "pet_tracker/heartrate"
gps_topic = "pet_tracker/gps"
activity_topic = "pet_tracker/activity"
collar_topic = "pet_tracker/collar"
powerstatus = "pet_tracker/powerstatus"

# MQTT Client
client = mqtt.Client()

def on_message(client, userdata, message):
    # Decode message payload and extract data
    payload_str = message.payload.decode('utf-8')

    try:
        payload = json.loads(payload_str)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON payload: {e}")
        return

    timestamp = timezone.now()

    # Extract topic and sensor type from the message
    topic = message.topic
    sensor_type = topic.split('/')[-1]

    # Print received message details
    print(f'Received message on topic: {topic} with payload: {payload}')


    if sensor_type == 'temperature':
        # Fetch the corresponding collar based on Serial Number
        try:
            collar = Collar.objects.get(SerialNumber=payload['serial_number'])
        except Collar.DoesNotExist:
            raise ValueError(f"Collar with Serial Number {payload['serial_number']} does not exist.")

        # Create temperature reading associated with the collar
        temperature_reading = TemperatureReading.objects.create(collar=collar, temperature=payload['temperature'])
        print(f'Temperature reading saved: {temperature_reading}')

        serial = None
        docs =(
        db.collection('pet_table')
        .where(filter=FieldFilter("user", "==", "UqoaFk1MldRA9TBfXN1rG3jDPEW2"))
        .stream()
        )
        for doc in docs:
            doc_data = doc.to_dict()
            serial = doc_data.get('serial_number') 


        db.collection("temperature_reading").document().set({
        'temperature' : payload['temperature'],
        'serial_number': serial,
        'time': datetime.now(),
        })
  

    elif sensor_type == 'heartrate':
        # Create heart rate reading associated with the pet
        try:
            collar = Collar.objects.get(SerialNumber=payload['serial_number'])
        except Collar.DoesNotExist:
            raise ValueError(f"Collar with Serial Number {payload['serial_number']} does not exist.")
        heart_rate_reading = HeartRateReading.objects.create(
            collar=collar,
            HeartRate=payload['heart_rate'],
            SPO2=payload['spo2']
        )
        print(f'Heart rate reading saved: {heart_rate_reading}')

    elif sensor_type == 'gps':
        try:
            collar = Collar.objects.get(SerialNumber=payload['serial_number'])
        except Collar.DoesNotExist:
            raise ValueError(f"Collar with Serial Number {payload['serial_number']} does not exist.")
        gps_reading = GPSReading.objects.create(
            collar=collar,
            longitude=payload['longitude'],
            latitude=payload['latitude']
        )
        print(f'GPS reading saved: {gps_reading}')

    elif sensor_type == 'HeartRateVariability':
        try:
            collar = Collar.objects.get(SerialNumber=payload['serial_number'])
        except Collar.DoesNotExist:
            raise ValueError(f"Collar with Serial Number {payload['serial_number']} does not exist.")
        HRV_readings= HeartRateVariability.objects.create(
            collar=collar,
            HRV=payload['hrv'],
        )
        print(f'HeartRateVariability reading saved: {HRV_readings}')
    
    elif sensor_type == 'activity':
        try:
            collar = Collar.objects.get(SerialNumber=payload['serial_number'])
        except Collar.DoesNotExist:
            raise ValueError(f"Collar with Serial Number {payload['serial_number']} does not exist.")
        activity_reading = ActivityLevel.objects.create(
            collar=collar,
            activity_x=payload['activity']['x'],
            activity_y=payload['activity']['y'],
            activity_z=payload['activity']['z']
        )
        print(f'Activity level reading saved: {activity_reading}')

    elif sensor_type == 'collar':
        # Create collar battery message
        message_warning = f"Smart pet collar is going off: {payload['message']}"
        CollarBattery.objects.create(timestamp=timestamp, message=message_warning)
        print(f'Collar battery message saved: {message_warning}')
    else:
        print(f'Unknown sensor type: {sensor_type}')

def publish_message(message):
    result = client.publish(powerstatus, message)
    status = result[0]
    if status == 0:
        print(f"Sent `{message}` to topic `{powerstatus}`")
    else:
        print(f"Failed to send message to topic {powerstatus}")


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully to MQTT broker" + str(rc))
        client.subscribe(temperature_topic)
        client.subscribe(heartrate_topic)
        client.subscribe(gps_topic)
        client.subscribe(activity_topic)
        client.subscribe(collar_topic)
    else:
        print("Could not connect to MQTT broker. Return code:", rc)

# Setup MQTT client
client.on_connect = on_connect
client.on_message = on_message

try:
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    client.connect(MQTT_SERVER, MQTT_PORT)
    client.loop_forever()
except KeyboardInterrupt:
    print("Disconnecting from MQTT Broker...")
    client.disconnect()
