from transformers import AutoModelForCTC, Wav2Vec2Processor
import os

from dotenv import load_dotenv
load_dotenv('/home/ubuntu/ws/src/.env')

os.environ.get('DATA_PATH')

model_download = "patrickvonplaten/wav2vec2-base-timit-demo-google-colab"
model_path = f"{ os.environ.get('DATA_PATH') }models/wav2vec2-full"

model = AutoModelForCTC.from_pretrained(model_download)
processor = Wav2Vec2Processor.from_pretrained(model_download)


if os.path.exists(model_path) == False:
    os.mkdir(model_path)
    model.save_pretrained(model_path)
    processor.save_pretrained(model_path)

#loads the finetuned wav2vec2 model from hugginface and save it in the model_path location to be used offline