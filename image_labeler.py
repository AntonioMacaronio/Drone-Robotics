import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import json

class ImageLabelingTool:
    def __init__(self, master):
        self.master = master
        self.master.title("Image Labeling Tool")

        self.canvas = tk.Canvas(master)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.upload_button = tk.Button(master, text="Upload Image", command=self.upload_image)
        self.upload_button.pack()

        self.save_button = tk.Button(master, text="Save Labels", command=self.save_labels, state=tk.DISABLED)
        self.save_button.pack()

        self.image = None
        self.photo = None
        self.bounding_box = None
        self.start_x = None
        self.start_y = None
        self.current_rect = None
        self.labels = []

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

    def upload_image(self):
        file_path = filedialog.askopenfilename()
        if file_path:
            self.image = Image.open(file_path)
            self.photo = ImageTk.PhotoImage(self.image)
            self.canvas.config(width=self.image.width, height=self.image.height)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
            self.save_button.config(state=tk.NORMAL)

    def on_click(self, event):
        self.start_x = self.canvas.canvasx(event.x)
        self.start_y = self.canvas.canvasy(event.y)
        self.current_rect = self.canvas.create_rectangle(self.start_x, self.start_y, self.start_x, self.start_y, outline='red')

    def on_drag(self, event):
        cur_x = self.canvas.canvasx(event.x)
        cur_y = self.canvas.canvasy(event.y)
        self.canvas.coords(self.current_rect, self.start_x, self.start_y, cur_x, cur_y)

    def on_release(self, event):
        end_x = self.canvas.canvasx(event.x)
        end_y = self.canvas.canvasy(event.y)
        points = {
            'x1': min(self.start_x, end_x),
            'y1': min(self.start_y, end_y),
            'x2': max(self.start_x, end_x),
            'y2': max(self.start_y, end_y),
        }
        self.labels.append(points)
        print(f"Bounding box added: {self.labels[-1]}")
        # print(min(self.start_x, end_x) / 480.0, min(self.start_y, end_y) / 640.0, max(self.start_x, end_x) / 480.0, max(self.start_y, end_y) / 640.0)
        print((points['x1'] + points['x2']) / 960.0, (points['y1'] + points['y2']) / 1280.0, (points['x2'] - points['x1']) / 480.0, (points['y2'] - points['y1']) / 640.0)


    def save_labels(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".json")
        if file_path:
            with open(file_path, 'w') as f:
                json.dump(self.labels, f)
            print(f"Labels saved to {file_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ImageLabelingTool(root)
    root.mainloop()