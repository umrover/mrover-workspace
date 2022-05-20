import boto3
import os
import zipfile
import json


def main():
    if not os.path.exists("base_station/download_map/src/keys.json"):
        out = {"accessKey": "", "secretKey": ""}
        f = open("base_station/download_map/src/keys.json", "w")
        json.dump(out, f, indent=4)
        print("Access Keys not found, input keys into keys.json")
        exit(0)
    f = open("base_station/download_map/src/keys.json", "r")
    keys = json.load(f)
    s3 = boto3.client('s3', aws_access_key_id=keys["accessKey"],
                      aws_secret_access_key=keys["secretKey"])
    print("Downloading map...")
    s3.download_file("rover-map", "map.zip", "base_station/gui/src/static/map.zip")
    print("Map Download Complete, Unzipping...")
    with zipfile.ZipFile("base_station/gui/src/static/map.zip", 'r') as zip_ref:
        zip_ref.extractall("base_station/gui/src/static/map")
    os.remove("base_station/gui/src/static/map.zip")
    print("Map Download Complete!")
