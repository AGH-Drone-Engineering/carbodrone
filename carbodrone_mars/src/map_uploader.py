import json
import random
import string
import firebase_admin
from firebase_admin import credentials, firestore
import argparse
import os
import base64

# Check if key.json exists, if not, create it by decoding basekey.json
def ensure_key_file():
    key_file = 'key.json'
    if not os.path.exists(key_file):
        with open('basekey.json', 'r') as basekey_file:
            encoded_key = basekey_file.read()
            decoded_key = base64.b64decode(encoded_key)
            
            # Write decoded content to key.json
            with open(key_file, 'wb') as key_file_out:
                key_file_out.write(decoded_key)
        print(f"key.json created from basekey.json")
    else:
        print(f"key.json already exists")

# Initialize Firebase Admin SDK
def initialize_firebase():
    cred = credentials.Certificate("key.json")
    firebase_admin.initialize_app(cred)
    return firestore.client()

# Upload points to Firestore
def upload_points_to_firestore(data, db):
    for object in data:
        name = object["name"]
        lat = object["lat"]
        lng = object["lon"]
        
        # Generate random ID
        document_id = ''.join(random.choices(string.ascii_lowercase + string.digits, k=20))
        
        # Create Firestore document
        doc_ref = db.collection('mines-points').document(document_id)
        
        # Data to be inserted
        doc_data = {
            'location': firestore.GeoPoint(lat, lng),
            'type': name,
            'alt': 4,
            'shooted': False,
            'image': ""
        }
        
        # Upload to Firestore
        doc_ref.set(doc_data)
        print(f"Uploaded point for color '{name}' at {lat}, {lng} to Firestore with ID {document_id}")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Upload JSON points to Firestore")
    parser.add_argument("json_string")
    
    args = parser.parse_args()
    
    # Ensure the Firebase key file exists
    ensure_key_file()
    
    # Initialize Firestore
    db = initialize_firebase()
    
    # Load JSON data from the provided file path
    data = json.loads(args.json_string)
    
    # Upload points to Firestore
    upload_points_to_firestore(data, db)
