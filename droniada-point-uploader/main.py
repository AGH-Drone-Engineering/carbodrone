import json
import random
import string
import firebase_admin
from firebase_admin import credentials, firestore
import argparse

# Initialize Firebase Admin SDK
cred = credentials.Certificate("key.json")
firebase_admin.initialize_app(cred)
db = firestore.client()

# Load and parse the JSON file
def load_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

# Upload points to Firestore
def upload_points_to_firestore(data):
    for color, location in data.items():
        lat, lng = location
        
        # Generate random ID
        document_id = ''.join(random.choices(string.ascii_lowercase + string.digits, k=20))
        
        # Create Firestore document
        doc_ref = db.collection('mines-points').document(document_id)
        
        # Data to be inserted
        doc_data = {
            'location': firestore.GeoPoint(lat, lng),
            'type': color,
            'alt': 4,
            'shooted': False,
            'image': ""
        }
        
        # Upload to Firestore
        doc_ref.set(doc_data)
        print(f"Uploaded point for color '{color}' at {lat}, {lng} to Firestore with ID {document_id}")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Upload JSON points to Firestore")
    parser.add_argument("json_file_path", help="Path to the JSON file")
    
    args = parser.parse_args()
    
    # Load JSON data from the provided file path
    data = load_json(args.json_file_path)
    
    # Upload points to Firestore
    upload_points_to_firestore(data)
