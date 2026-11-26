import sys
from pathlib import Path

from openpyxl import load_workbook

for path in Path(sys.argv[1]).glob('*参数*.xlsx'):
    print(f'===== {path.name} =====')
    book = load_workbook(path, data_only=True)
    for sheet in book:
        print(f'--- {sheet.title} ({sheet.max_row} × {sheet.max_column}) ---')
        limit = sheet.max_row if sheet.max_row < 55 else 12
        for row in sheet.iter_rows(max_row=limit, values_only=True):
            if any(value is not None for value in row):
                print(' | '.join('' if value is None else str(value) for value in row).rstrip(' |'))
