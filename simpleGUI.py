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

# 设置使用内部模型
model_config.__use_inside_model__ = True

class PDFToMarkdownConverter:
    def __init__(self, root):
        self.root = root
        self.root.title("PDF to Markdown Converter")

        self.config_path = None
        self.pdf_path = None

        # 配置变量
        self.bucket_info = StringVar()
        self.temp_output_dir = StringVar()
        self.models_dir = StringVar()
        self.device_mode = StringVar()

        # 设置GUI
        self.setup_gui()

        # 设置日志
        self.setup_logging()

    def setup_gui(self):
        # 选择配置文件按钮
        Button(self.root, text="选择 magic-pdf.json 文件", command=self.select_magic_pdf_file).pack(pady=10)

        # 配置文件标签
        self.config_label = Label(self.root, text="未选择配置文件")
        self.config_label.pack(pady=10)

        # 配置输入字段
        self.create_labeled_entry("Bucket 信息:", self.bucket_info)
        self.create_labeled_entry("临时输出目录:", self.temp_output_dir, self.select_temp_output_dir)
        self.create_labeled_entry("模型目录:", self.models_dir, self.select_models_dir)

        Label(self.root, text="设备模式:").pack(pady=5)
        OptionMenu(self.root, self.device_mode, "cuda", "cpu").pack(pady=5)

        # 保存配置按钮
        Button(self.root, text="保存配置", command=self.save_config).pack(pady=10)

        # 选择PDF文件按钮
        Button(self.root, text="选择PDF文件", command=self.select_file).pack(pady=10)

        # 文件标签
        self.file_label = Label(self.root, text="未选择文件")
        self.file_label.pack(pady=10)

        # 转换按钮
        Button(self.root, text="转换为Markdown", command=self.start_conversion).pack(pady=10)

        # 结果标签
        self.result_label = Label(self.root, text="")
        self.result_label.pack(pady=10)

        # 日志文本框
        self.log_text = ScrolledText(self.root, height=10)
        self.log_text.pack(pady=10, fill='both', expand=True)

    def setup_logging(self):
        # 设置日志记录器
        logger.remove()
        logger.add(self.log_text_write, level="DEBUG")

        # 重定向标准输出和标准错误
        sys.stdout = self.log_text_writer()
        sys.stderr = self.log_text_writer()

    def log_text_write(self, message):
        # 将日志消息写入文本框
        self.log_text.insert(END, message)
        self.log_text.see(END)  # 自动滚动到末尾

    def log_text_writer(self):
        # 创建一个文本重定向器类
        class TextRedirector:
            def __init__(self, text_widget):
                self.text_widget = text_widget

            def write(self, message):
                self.text_widget.insert(END, message)
                self.text_widget.see(END)

            def flush(self):
                pass  # 如果代码调用 sys.stdout.flush()，则需要此方法

        return TextRedirector(self.log_text)

    def create_labeled_entry(self, label_text, var, button_command=None):
        # 创建带标签的输入框
        Label(self.root, text=label_text).pack(pady=5)
        if button_command:
            Button(self.root, text="选择目录", command=button_command).pack(pady=5)
        Entry(self.root, textvariable=var, width=50).pack(pady=5)

    def select_magic_pdf_file(self):
        # 选择配置文件
        self.config_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if self.config_path:
            with open(self.config_path, 'r') as f:
                config_data = json.load(f)
            self.bucket_info.set(json.dumps(config_data.get("bucket_info", {}), indent=4))
            self.temp_output_dir.set(config_data.get("temp-output-dir", ""))
            self.models_dir.set(config_data.get("models-dir", ""))
            self.device_mode.set(config_data.get("device-mode", "cpu"))
            self.config_label.config(text=f"已选择配置文件: {self.config_path}")

    def save_config(self):
        # 保存配置文件
        config_data = {
            "bucket_info": json.loads(self.bucket_info.get()),
            "temp-output-dir": self.temp_output_dir.get(),
            "models-dir": self.models_dir.get(),
            "device-mode": self.device_mode.get()
        }
        if self.config_path:
            with open(self.config_path, 'w') as f:
                json.dump(config_data, f, indent=4)
            self.result_label.config(text="配置已保存!")

    def select_file(self):
        # 选择PDF文件
        self.pdf_path = filedialog.askopenfilename(filetypes=[("PDF files", "*.pdf")])
        if self.pdf_path:
            self.file_label.config(text=f"已选择文件: {self.pdf_path}")
            self.temp_output_dir.set(os.path.dirname(self.pdf_path))  # 自动设置临时输出目录为PDF所在路径

    def start_conversion(self):
        # 启动转换线程
        conversion_thread = threading.Thread(target=self.convert_file)
        conversion_thread.start()

    def convert_file(self):
        # 转换文件
        if not self.pdf_path:
            self.result_label.config(text="未选择PDF文件!")
            return
    
        try:
            # 获取PDF文件所在目录
            pdf_dir = os.path.dirname(self.pdf_path)
            demo_name = os.path.splitext(os.path.basename(self.pdf_path))[0]
            pdf_bytes = open(self.pdf_path, "rb").read()
            
            model_json = []  # model_json传空list使用内置模型解析
            jso_useful_key = {"_pdf_type": "", "model_list": model_json}
            
            local_image_dir = os.path.join(pdf_dir, 'images')
            image_writer = DiskReaderWriter(local_image_dir)
            
            pipe = UNIPipe(pdf_bytes, jso_useful_key, image_writer)
            pipe.pipe_classify()
            
            if len(model_json) == 0:
                if model_config.__use_inside_model__:
                    pipe.pipe_analyze()
                else:
                    logger.error("需要模型列表输入")
                    return
            
            pipe.pipe_parse()
            md_content = pipe.pipe_mk_markdown(os.path.basename(local_image_dir), drop_mode="none")
            
            # 将Markdown文件保存在PDF文件的同一目录下
            md_file_path = os.path.join(pdf_dir, f"{demo_name}.md")
            with open(md_file_path, "w", encoding="utf-8") as f:
                f.write(md_content)
            
            self.result_label.config(text="转换成功!")
        except Exception as e:
            logger.exception("转换失败!")
            self.result_label.config(text="转换失败!")

    def select_temp_output_dir(self):
        # 选择临时输出目录
        directory = filedialog.askdirectory()
        if directory:
            self.temp_output_dir.set(directory)

    def select_models_dir(self):
        # 选择模型目录
        directory = filedialog.askdirectory()
        if directory:
            self.models_dir.set(directory)

# 运行应用程序
if __name__ == "__main__":
    root = Tk()
    app = PDFToMarkdownConverter(root)
    root.mainloop()