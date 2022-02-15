#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import argparse


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--apriltag_directory', type=str, default='apriltag-imgs-master/tag36h11')
    parser.add_argument('--start_number', type=int, default=0)
    parser.add_argument('--stop_number', type=int, default=500)

    args = parser.parse_args()

    latex_content = '''
\\documentclass[11pt,a4paper]{jsarticle}
\\usepackage[margin=15mm]{geometry}
\\setlength\\parindent{0pt}
\\usepackage[dvipdfmx]{graphicx}
\\begin{document}'''

    for num in range(args.start_number, args.stop_number):
        latex_content += '''
\\begin{{center}}
実験用に貼り付けています。
Spot Fiducial ID: {:0=5}

東京大学情報システム工学研究室\\\\
連絡先(TEL): 03-5841-7416\\\\
連絡先(E-Mail): spot@jsk.imi.i.u-tokyo.ac.jp\\\\
\\end{{center}}

\\begin{{figure}}[b]
    \\centering
    \\includegraphics[keepaspectratio,width=182.5mm]{{{}/tag36_11_{:0=5}.png}}
\\end{{figure}}

\\newpage
\\clearpage
'''.format(num, args.apriltag_directory, num)

    latex_content += '\\end{document}'

    print(latex_content)
