from vosk import Model, KaldiRecognizer
import pyaudio
from djitellopy import Tello

model = Model(model_path="/Users/sshashi/Desktop/Personal/Hackathons/Drone-Robotics/vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(model, 16000)

mic = pyaudio.PyAudio()

def get_command():
    stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
    stream.start_stream()
    while True:
        try:
            data = stream.read(4096)
            if recognizer.AcceptWaveform(data):
                result = recognizer.Result()
                response = result[14:-3]
                stream.close()
                return response
        except OSError:
            pass

def analyze_command(command):
    try:
        if command == "take off":
            tello.takeoff()
        if command == "land":
            tello.land()
    except Exception:
        pass

if __name__ == '__main__':
    tello = Tello()
    tello.connect()
    tello.streamon()

    tello.takeoff()
    tello.rotate_clockwise(90)
    try:
        while True:
            command = get_command()
            analyze_command(command)
            

    except KeyboardInterrupt:
        exit(1)
    finally:
        print("fin")
