# Project Setup and Installation Guide

## 1. Installation of XAMPP (if required)
XAMPP is used as a local server environment for testing purposes.

1. Download XAMPP from the official website: [XAMPP Download](https://www.apachefriends.org/download.html)
2. Install XAMPP on your PC.
3. After installation, you can open XAMPP to view and manage your local server environment (e.g., Apache, MySQL).

## 2. Backend Setup with Django

### Step 1: Install Required Libraries
The necessary libraries for the Django backend are listed in the `requirements.txt` file.

1. Open your terminal or command prompt.
2. Navigate to the directory containing `requirements.txt`.
3. Run the following command to install the libraries:

   ```sh
   pip install -r requirements.txt

### Step 2: Database Configuration with Firebase
This project uses Firebase as the database.

Create a Firebase project in the Firebase Console.
Set up Firebase Realtime Database or Firestore, depending on your project needs.
Obtain the Firebase configuration (API key, database URL, etc.).
Add Firebase to your settings.py or create a separate configuration file to manage Firebase settings in your Django project.

### Step 3: Running the Django Server
Apply migrations by running:

copy this code ; **python manage.py migrate**
If you have new or modified models, create migrations with:

Copy code ; **python manage.py makemigrations**
Start the Django development server using:

Copy code; **python manage.py runserver**

### Step 4: MQTT Broker Setup
The project includes an mqttbroker.py file for handling MQTT (Message Queuing Telemetry Transport) data from hardware.

Install the paho-mqtt library:

copy this in the terminal ; **pip install paho-mqtt**
Run the MQTT broker script separately from the Django server:
 copy this to terminal ; **python mqttbroker.py**

## 3. Frontend Setup
### Step 1: Install Required Libraries
The frontend requires certain libraries found in the lib folder or specified in the frontend documentation.

Follow the instructions in the kelpet-frontend directory to install these libraries.
## 4. Hardware Setup
### Step 1: Configure the ESP32 Hardware
Download and install the EspSmartConfig app from the Play Store to configure your ESP32 hardware.
Ensure that your hardware is powered on before attempting to connect it to your Wi-Fi network.

### Step 2: Retrieve the Collar Serial Number
The mobile app requires the collar serial number, which can be found in the kelpet-esp32code directory.

Follow the instructions provided in the YouTube tutorial for the smartconfig: Watch Tutorial. https://youtu.be/xsxTBSn-OJo
### Step 3: Using Local Mosquitto for MQTT
If you're using a local Mosquitto MQTT broker:

Install paho-mqtt for your pc. 