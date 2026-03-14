import os
from docx import Document

SOURCE_DOCX = '/Users/zclb00103397/Documents/01_Code_Qiyuan/interface/330二开接口列表.docx'
OUTPUT_MD = '/Users/zclb00103397/Documents/01_Code_Qiyuan/interface/primebot_sdk/Dev_Tools/330二开接口列表.md'

def extract_interface_names():
    """Extract only interface names from source docx tables."""
    src = Document(SOURCE_DOCX)
    interface_names = []
    HEADER_COL_0 = '接口名称'

    for table in src.tables:
        for row in table.rows:
            cell_text = row.cells[0].text.strip()
            if cell_text and cell_text != HEADER_COL_0:
                if cell_text not in interface_names:
                    interface_names.append(cell_text)
                    
    return interface_names

def create_md(interface_names):
    with open(OUTPUT_MD, 'w', encoding='utf-8') as f:
        f.write("# 330二开接口列表\n\n")
        f.write(f"共计 {len(interface_names)} 个接口名称：\n\n")
        for name in interface_names:
            f.write(f"- {name}\n")
    print(f"成功生成: {OUTPUT_MD}")

if __name__ == '__main__':
    if os.path.exists(SOURCE_DOCX):
        names = extract_interface_names()
        create_md(names)
    else:
        print(f"错误: 找不到源文件 {SOURCE_DOCX}")
