#!/usr/bin/env python3
"""Small ArUco tags for the thin rod + a base tag for the zero mark. Print at 100 % (Actual size)."""
import cv2
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
W, H = 210.0, 297.0
c = canvas.Canvas("small_tags.pdf", pagesize=(W * mm, H * mm))


def tag(x, y, tid, size, label=True):
    bits = cv2.aruco.generateImageMarker(d, tid, 6, borderBits=1) < 128
    cell = size / 6
    c.setFillColorRGB(0, 0, 0)
    for i in range(6):
        for j in range(6):
            if bits[i, j]:
                c.rect((x + j * cell) * mm, (H - y - (i + 1) * cell) * mm, cell * mm, cell * mm, stroke=0, fill=1)
    q = max(2.0, size * 0.25)           # white quiet zone to keep when cutting
    c.setStrokeColorRGB(0.75, 0.75, 0.75)
    c.setLineWidth(0.2)
    c.rect((x - q) * mm, (H - y - size - q) * mm, (size + 2 * q) * mm, (size + 2 * q) * mm, stroke=1, fill=0)
    if label:
        c.setFillColorRGB(0.3, 0.3, 0.3)
        c.setFont("Helvetica", 5)
        c.drawString((x - q) * mm, (H - y - size - q - 2.4) * mm, f"ID{tid} {size:g}mm")


c.setFont("Helvetica-Bold", 10)
c.drawString(12 * mm, (H - 12) * mm, "Small tags for rod components + BASE tag  |  DICT_4X4_50  |  print at 100% (Actual size), laser printer")
c.setFont("Helvetica", 8)
c.drawString(12 * mm, (H - 18) * mm, "Keep the white border (grey line) when cutting. Stick on a small FLAT backing, never wrapped round the rod.")
y = 30
for size in (6.0, 8.0, 10.0):
    c.setFont("Helvetica-Bold", 9); c.setFillColorRGB(0, 0, 0)
    c.drawString(12 * mm, (H - y + 4) * mm, f"{size:g} mm tags (IDs 30-37)")
    for k, tid in enumerate(range(30, 38)):
        tag(14 + k * (size + 2 * max(2, size * 0.25) + 6), y, tid, size)
    y += size + 22
c.setFont("Helvetica-Bold", 9); c.setFillColorRGB(0, 0, 0)
c.drawString(12 * mm, (H - y + 4) * mm, "BASE tag ID 45: stick on the column exactly at the bench zero mark (its centre = the mark)")
y += 8
tag(14, y, 45, 15.0); tag(50, y, 45, 20.0)
c.setStrokeColorRGB(1, 0, 0); c.setLineWidth(0.3)
for x0, s in ((14, 15.0), (50, 20.0)):      # centre cross-hair guides on the white border
    cy = y + s / 2
    c.line((x0 - 3.5) * mm, (H - cy) * mm, (x0 - 1) * mm, (H - cy) * mm)
    c.line((x0 + s + 1) * mm, (H - cy) * mm, (x0 + s + 3.5) * mm, (H - cy) * mm)
y += 34
c.setFont("Helvetica", 8); c.setFillColorRGB(0, 0, 0)
c.drawString(12 * mm, (H - y) * mm, "Size check: this bar must measure exactly 50.0 mm")
c.setFillColorRGB(0, 0, 0); c.rect(12 * mm, (H - y - 6) * mm, 50 * mm, 1.5 * mm, stroke=0, fill=1)
c.showPage(); c.save()
print("saved small_tags.pdf")
