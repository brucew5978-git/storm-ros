from transformers import AutoModelForCTC, Wav2Vec2Processor
import torch, torchaudio

import os
from dotenv import load_dotenv
load_dotenv('/home/ubuntu/ws/src/.env')

default_audio_path = f"{ os.environ.get('DATA_PATH') }audio/input.wav"
default_model_path = f"{ os.environ.get('DATA_PATH') }models/wav2vec2-full"

class Inference_Model():
    def __init__(self, model_path=default_model_path):
        self.model = None
        self.processor = None
        self.model_loaded = False

        self.load_model(model_path)


    def load_model(self, model_path=default_model_path):
        try:
            self.model = AutoModelForCTC.from_pretrained(model_path)
            self.processor = Wav2Vec2Processor.from_pretrained(model_path)

            self.model_loaded = True
        except:
            print(f"Error: could not load model from: { model_path }")

    #performs inference on batch and returns results
    def map_to_result(self, batch_tensor):
        if not self.model_loaded:
            print('Error: no model loaded, cannot perform inference')

            return ""
        # torch.no_grad() ensures no gradients are changed during inferencing
        with torch.no_grad():
            logits = self.model(batch_tensor).logits

        pred_ids = torch.argmax(logits, dim=-1)
        predicted_text = self.processor.batch_decode(pred_ids)[0]

        return predicted_text

    def infer_text(self, audio_path=default_audio_path):
        if not self.model_loaded:
            print('Error: no model loaded, cannot perform inference')

            return ""
        
        print(f"infering audio found at: { audio_path }")

        try:
            # verify if sample rate of model corresponds to wav file sample rate
            waveform, file_sample_rate = torchaudio.load(audio_path, normalize=True) 
            target_sample_rate = self.processor.feature_extractor.sampling_rate
        except:
            print(f"Error: could not load audio: { audio_path }")
            return ""

        if file_sample_rate != target_sample_rate:
            resampler= torchaudio.transforms.Resample(file_sample_rate, target_sample_rate)
            waveform = resampler(waveform)

        # Preprocess input data
        input = self.processor(waveform, sampling_rate=target_sample_rate, return_tensors="pt", padding=True)
        input_tensor = input["input_values"].squeeze()


        predicted_text = self.map_to_result(input_tensor)
        print(f"predicted command: |{ predicted_text }|")
        return predicted_text

if __name__ == '__main__':
    model = Inference_Model()
    model.infer_text()
