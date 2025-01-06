import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import threading

# 导入 tkinterdnd2 用于拖放支持
try:
    from tkinterdnd2 import DND_FILES, TkinterDnD
except ImportError:
    messagebox.showerror("缺少库", "请安装 tkinterdnd2 库: pip install tkinterdnd2")
    raise

from magic_pdf.data.data_reader_writer import FileBasedDataWriter, FileBasedDataReader
from magic_pdf.data.dataset import PymuDocDataset
from magic_pdf.model.doc_analyze_by_custom_model import doc_analyze
from magic_pdf.config.enums import SupportedPdfParseMethod

class PDFtoMarkdownGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PDF 转 Markdown 工具")
        self.root.geometry("700x500")
        self.root.resizable(False, False)  # 不允许拉伸窗口
        self.create_widgets()

    def create_widgets(self):
        style = ttk.Style()
        style.theme_use("clam")  # 设置主题
        style.configure(".", font=("Microsoft YaHei", 10))  # 设置全局字体
        style.configure("TLabelFrame", font=("Microsoft YaHei", 11, "bold"), padding=10)

        padding = {'padx': 10, 'pady': 10}

        # PDF文件选择区域
        self.pdf_path = tk.StringVar()
        pdf_frame = ttk.LabelFrame(self.root, text="选择PDF文件")
        pdf_frame.grid(row=0, column=0, columnspan=3, sticky='we', **padding)

        # 拖放区域
        self.drop_area = tk.Label(pdf_frame, text="将PDF文件拖放到这里", relief='ridge', width=60, height=5, bg='lightgray')
        self.drop_area.pack(padx=10, pady=10)
        self.drop_area.drop_target_register(DND_FILES)
        self.drop_area.dnd_bind('<<Drop>>', self.drop_pdf)

        # 或者使用按钮选择文件
        button_frame = tk.Frame(pdf_frame)
        button_frame.pack(pady=5)
        tk.Button(button_frame, text="浏览文件", command=self.browse_pdf).pack()

        # 输出目录选择
        self.output_dir = tk.StringVar()  # 原先为 "output"
        output_frame = ttk.LabelFrame(self.root, text="选择输出目录")
        output_frame.grid(row=1, column=0, columnspan=3, sticky='we', **padding)

        tk.Entry(output_frame, textvariable=self.output_dir, width=60).pack(side='left', padx=10, pady=10)
        tk.Button(output_frame, text="浏览", command=self.browse_output_dir).pack(side='left', padx=10, pady=10)

        # 转换按钮
        self.convert_button = tk.Button(self.root, text="开始转换", command=self.start_conversion, bg='blue', fg='white', font=('Arial', 12, 'bold'))
        self.convert_button.grid(row=2, column=1, **padding)

        # 进度显示
        self.progress = ttk.Progressbar(self.root, orient='horizontal', mode='indeterminate')
        self.progress.grid(row=3, column=0, columnspan=3, sticky='we', **padding)

        # 日志输出
        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.grid(row=4, column=0, columnspan=3, sticky='nsew', **padding)
        self.log_text = tk.Text(log_frame, height=15, state='disabled', wrap='word')
        self.log_text.pack(fill='both', expand=True, padx=10, pady=10)

        # 配置网格权重以使日志区域可扩展
        self.root.grid_rowconfigure(4, weight=1)
        self.root.grid_columnconfigure(2, weight=1)

    def browse_pdf(self):
        file_path = filedialog.askopenfilename(
            title="选择PDF文件",
            filetypes=[("PDF Files", "*.pdf")]
        )
        if file_path:
            self.pdf_path.set(file_path)
            self.output_dir.set(os.path.dirname(file_path))  # 新增：默认输出目录为PDF所在目录
            self.log(f"选择的PDF文件: {file_path}")

    def browse_output_dir(self):
        directory = filedialog.askdirectory(title="选择输出目录")
        if directory:
            self.output_dir.set(directory)
            self.log(f"选择的输出目录: {directory}")

    def drop_pdf(self, event):
        # 获取拖放的文件路径，支持多个文件
        files = self.root.splitlist(event.data)
        for file in files:
            if file.lower().endswith('.pdf'):
                self.pdf_path.set(file)
                self.output_dir.set(os.path.dirname(file))  # 新增：默认输出目录为PDF所在目录
                self.log(f"拖放的PDF文件: {file}")
                break
            else:
                self.log(f"忽略非PDF文件: {file}")

    def start_conversion(self):
        pdf_file = self.pdf_path.get()
        output_dir = self.output_dir.get()

        if not pdf_file:
            messagebox.showwarning("输入错误", "请先选择一个PDF文件。")
            return

        if not os.path.isfile(pdf_file):
            messagebox.showerror("文件错误", "选择的PDF文件不存在。")
            return

        if not pdf_file.lower().endswith('.pdf'):
            messagebox.showerror("文件类型错误", "请选择一个PDF文件。")
            return

        os.makedirs(output_dir, exist_ok=True)

        # 禁用按钮并启动进度条
        self.convert_button.config(state='disabled')
        self.progress.start()
        self.log("开始转换...")

        # 使用线程避免阻塞GUI
        threading.Thread(target=self.convert_pdf_to_md, args=(pdf_file, output_dir), daemon=True).start()

    def convert_pdf_to_md(self, pdf_file, output_dir):
        try:
            name_without_suff = os.path.splitext(os.path.basename(pdf_file))[0]
            local_image_dir = os.path.join(output_dir, "images")
            local_md_dir = output_dir
            os.makedirs(local_image_dir, exist_ok=True)

            image_writer = FileBasedDataWriter(local_image_dir)
            md_writer = FileBasedDataWriter(local_md_dir)

            reader = FileBasedDataReader("")
            pdf_bytes = reader.read(pdf_file)

            ds = PymuDocDataset(pdf_bytes)

            if ds.classify() == SupportedPdfParseMethod.OCR:
                self.log("使用OCR解析PDF...")
                infer_result = ds.apply(doc_analyze, ocr=True)
                pipe_result = infer_result.pipe_ocr_mode(image_writer)
            else:
                self.log("使用文本模式解析PDF...")
                infer_result = ds.apply(doc_analyze, ocr=False)
                pipe_result = infer_result.pipe_txt_mode(image_writer)

            # 绘制和保存结果
            infer_result.draw_model(os.path.join(local_md_dir, f"{name_without_suff}_model.pdf"))
            self.log(f"模型结果已保存: {name_without_suff}_model.pdf")

            pipe_result.draw_layout(os.path.join(local_md_dir, f"{name_without_suff}_layout.pdf"))
            self.log(f"布局结果已保存: {name_without_suff}_layout.pdf")

            pipe_result.draw_span(os.path.join(local_md_dir, f"{name_without_suff}_spans.pdf"))
            self.log(f"跨度结果已保存: {name_without_suff}_spans.pdf")

            pipe_result.dump_md(md_writer, f"{name_without_suff}.md", "images")
            self.log(f"Markdown 文件已保存: {name_without_suff}.md")

            pipe_result.dump_content_list(md_writer, f"{name_without_suff}_content_list.json", "images")
            self.log(f"内容列表已保存: {name_without_suff}_content_list.json")

            self.log("转换完成！")
            messagebox.showinfo("完成", "PDF 转 Markdown 转换完成。")
        except Exception as e:
            self.log(f"转换过程中发生错误: {e}")
            messagebox.showerror("错误", f"转换过程中发生错误:\n{e}")
        finally:
            self.progress.stop()
            self.convert_button.config(state='normal')

    def log(self, message):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

def main():
    # 使用 TkinterDnD 的 TkinterDnD.Tk 类代替 tk.Tk
    root = TkinterDnD.Tk()
    app = PDFtoMarkdownGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
