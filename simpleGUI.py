import os
import json
from tkinter import Tk, Button, Label, Entry, filedialog, StringVar, OptionMenu, Text, Scrollbar, END
from tkinter.scrolledtext import ScrolledText
from loguru import logger
from magic_pdf.pipe.UNIPipe import UNIPipe
from magic_pdf.rw.DiskReaderWriter import DiskReaderWriter
import magic_pdf.model as model_config
import sys
from contextlib import redirect_stdout, redirect_stderr
import threading

model_config.__use_inside_model__ = True

class PDFToMarkdownConverter:
    def __init__(self, root):
        self.root = root
        self.root.title("PDF to Markdown Converter")

        self.config_path = None
        self.pdf_path = None

        # Configuration variables
        self.bucket_info = StringVar()
        self.temp_output_dir = StringVar()
        self.models_dir = StringVar()
        self.device_mode = StringVar()

        # GUI setup
        self.setup_gui()

        # Log setup
        self.setup_logging()

    def setup_gui(self):
        # Select config button
        Button(self.root, text="Select magic-pdf.json File", command=self.select_magic_pdf_file).pack(pady=10)

        # Config label
        self.config_label = Label(self.root, text="No config file selected")
        self.config_label.pack(pady=10)

        # Configuration input fields
        self.create_labeled_entry("Bucket Info:", self.bucket_info)
        self.create_labeled_entry("Temp Output Dir:", self.temp_output_dir, self.select_temp_output_dir)
        self.create_labeled_entry("Models Dir:", self.models_dir, self.select_models_dir)

        Label(self.root, text="Device Mode:").pack(pady=5)
        OptionMenu(self.root, self.device_mode, "cuda", "cpu").pack(pady=5)

        # Save config button
        Button(self.root, text="Save Configuration", command=self.save_config).pack(pady=10)

        # Select PDF file button
        Button(self.root, text="Select PDF File", command=self.select_file).pack(pady=10)

        # File label
        self.file_label = Label(self.root, text="No file selected")
        self.file_label.pack(pady=10)

        # Convert button
        Button(self.root, text="Convert to Markdown", command=self.start_conversion).pack(pady=10)

        # Result label
        self.result_label = Label(self.root, text="")
        self.result_label.pack(pady=10)

        # Log text box
        self.log_text = ScrolledText(self.root, height=10)
        self.log_text.pack(pady=10, fill='both', expand=True)

    def setup_logging(self):
        logger.remove()
        logger.add(self.log_text_write, level="DEBUG")

        # Redirect stdout and stderr
        sys.stdout = self.log_text_writer()
        sys.stderr = self.log_text_writer()

    def log_text_write(self, message):
        self.log_text.insert(END, message)
        self.log_text.see(END)  # Auto-scroll to the end

    def log_text_writer(self):
        class TextRedirector:
            def __init__(self, text_widget):
                self.text_widget = text_widget

            def write(self, message):
                self.text_widget.insert(END, message)
                self.text_widget.see(END)

            def flush(self):
                pass  # This can be required if your code calls sys.stdout.flush()

        return TextRedirector(self.log_text)

    def create_labeled_entry(self, label_text, var, button_command=None):
        Label(self.root, text=label_text).pack(pady=5)
        if button_command:
            Button(self.root, text="Select Directory", command=button_command).pack(pady=5)
        Entry(self.root, textvariable=var, width=50).pack(pady=5)

    def select_magic_pdf_file(self):
        self.config_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if self.config_path:
            with open(self.config_path, 'r') as f:
                config_data = json.load(f)
            self.bucket_info.set(json.dumps(config_data.get("bucket_info", {}), indent=4))
            self.temp_output_dir.set(config_data.get("temp-output-dir", ""))
            self.models_dir.set(config_data.get("models-dir", ""))
            self.device_mode.set(config_data.get("device-mode", "cpu"))
            self.config_label.config(text=f"Selected config file: {self.config_path}")

    def save_config(self):
        config_data = {
            "bucket_info": json.loads(self.bucket_info.get()),
            "temp-output-dir": self.temp_output_dir.get(),
            "models-dir": self.models_dir.get(),
            "device-mode": self.device_mode.get()
        }
        if self.config_path:
            with open(self.config_path, 'w') as f:
                json.dump(config_data, f, indent=4)
            self.result_label.config(text="Configuration saved!")

    def select_file(self):
        self.pdf_path = filedialog.askopenfilename(filetypes=[("PDF files", "*.pdf")])
        if self.pdf_path:
            self.file_label.config(text=f"Selected file: {self.pdf_path}")
            self.temp_output_dir.set(os.path.dirname(self.pdf_path))  # 自动设置 Temp Output Dir 为 PDF 所在路径

    def start_conversion(self):
        conversion_thread = threading.Thread(target=self.convert_file)
        conversion_thread.start()

    def convert_file(self):
        if not self.pdf_path:
            self.result_label.config(text="No PDF file selected!")
            return

        try:
            current_script_dir = os.path.dirname(os.path.abspath(__file__))
            demo_name = os.path.splitext(os.path.basename(self.pdf_path))[0]
            pdf_bytes = open(self.pdf_path, "rb").read()
            
            model_json = []  # model_json传空list使用内置模型解析
            jso_useful_key = {"_pdf_type": "", "model_list": model_json}
            
            local_image_dir = os.path.join(current_script_dir, 'images')
            image_writer = DiskReaderWriter(local_image_dir)
            
            pipe = UNIPipe(pdf_bytes, jso_useful_key, image_writer)
            pipe.pipe_classify()
            
            if len(model_json) == 0:
                if model_config.__use_inside_model__:
                    pipe.pipe_analyze()
                else:
                    logger.error("Need model list input")
                    return
            
            pipe.pipe_parse()
            md_content = pipe.pipe_mk_markdown(os.path.basename(local_image_dir), drop_mode="none")
            
            with open(f"{demo_name}.md", "w", encoding="utf-8") as f:
                f.write(md_content)
            
            self.result_label.config(text="Conversion successful!")
        except Exception as e:
            logger.exception("Conversion failed!")
            self.result_label.config(text="Conversion failed!")

    def select_temp_output_dir(self):
        directory = filedialog.askdirectory()
        if directory:
            self.temp_output_dir.set(directory)

    def select_models_dir(self):
        directory = filedialog.askdirectory()
        if directory:
            self.models_dir.set(directory)

# Run the application
if __name__ == "__main__":
    root = Tk()
    app = PDFToMarkdownConverter(root)
    root.mainloop()
