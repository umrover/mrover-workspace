import boto3
import os
import zipfile

s3 = boto3.client('s3', aws_access_key_id="AKIAQNYJPFWU5SAZ4T5K",
                  aws_secret_access_key="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")


def main():
    print("Downloading map...")
    s3.download_file("rover-map", "map.zip", "base_station/gui/src/static/map.zip")
    print("Map Download Complete, Unzipping...")
    with zipfile.ZipFile("base_station/gui/src/static/map.zip", 'r') as zip_ref:
        zip_ref.extractall("base_station/gui/src/static/map")
    os.remove("base_station/gui/src/static/map.zip")
    print("Map Download Complete!")
