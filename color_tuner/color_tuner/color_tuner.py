import sys, yaml
import rclpy
from rclpy.node import Node
from cube_interfaces.msg import ColorRange, ColorRangeArray

from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QFileDialog,
    QMessageBox, QScrollArea, QSizePolicy, QSpinBox
)
from python_qt_binding.QtCore import Qt

class ColorTuner(Node, QWidget):
    '''
    GUI for easily tuning hsv color ranges, publishes colors to topic /colors after every change
    '''
    def __init__(self):
        Node.__init__(self, 'color_tuner')
        QWidget.__init__(self)
        self.pub = self.create_publisher(ColorRangeArray, '/colors', 10)

        self.entries = []  # list of (name_edit, [6 spinboxes])
        self.main_layout = QVBoxLayout()

        # Create Scroll area for dynamic list
        scroll = QScrollArea()
        self.container = QWidget()
        self.container_layout = QVBoxLayout()
        self.container.setLayout(self.container_layout)
        scroll.setWidget(self.container)
        scroll.setWidgetResizable(True)
        scroll.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.main_layout.addWidget(scroll)

        # Create buttons for add/remove/load/save
        btn_row = QHBoxLayout()
        for (txt, fn) in [('Add Color', self.add_color),
                          ('Remove Last', self.remove_color),
                          ('Load…', self.load_file),
                          ('Save…', self.save_file)]:
            b = QPushButton(txt)
            b.clicked.connect(fn)
            btn_row.addWidget(b)
        self.main_layout.addLayout(btn_row)

        self.setLayout(self.main_layout)
        self.setWindowTitle('Color Tuner')

        # start with one entry
        self.add_color()

        self.show()

    def add_color(self):
        idx = len(self.entries)
        row = QHBoxLayout()
        name = QLineEdit(f'color_{idx}')
        name.setFixedWidth(100)
        row.addWidget(name)

        spinboxes = []
        for i, label in enumerate(['Hl','Sl','Vl','Hu','Su','Vu']):
            col = QVBoxLayout()
            col.addWidget(QLabel(label))

            # create spinbox
            sb = QSpinBox()
            sb.setRange(0,179 if i % 3 == 0 else 255)
            sb.setValue(0 if i < 3 else sb.maximum())
            sb.setFixedWidth(60)
            sb.editingFinished.connect(self.publish_all)

            col.addWidget(sb)
            row.addLayout(col)
            spinboxes.append(sb)

        self.container_layout.addLayout(row)
        self.entries.append((name, spinboxes))
        self.publish_all()

    def remove_color(self):
        if not self.entries:
            return
        # remove last
        last_idx = self.container_layout.count() - 1
        item = self.container_layout.takeAt(last_idx)
        self.clear_layout(item.layout())
        self.entries.pop()
        self.publish_all()

    def clear_layout(self, layout):
        if layout is None:
            return
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
            else:
                self.clear_layout(item.layout())

    def publish_all(self, *_):
        arr = ColorRangeArray()
        for name, spinboxes in self.entries:
            cr = ColorRange()
            cr.name = name.text()
            values = [sb.value() for sb in spinboxes]
            cr.lower = values[0:3]
            cr.upper = values[3:6]
            arr.colors.append(cr)
        self.pub.publish(arr)

    def load_file(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Load colors', '', 'YAML Files (*.yaml)')
        if not path:
            return
        try:
            data = yaml.safe_load(open(path))
            # expect list of { name:, lower:, upper: }
            # first clear all
            while self.entries:
                self.remove_color()
            for item in data:
                self.add_color()
                name, spinboxes = self.entries[-1]
                name.setText(item.get('name',''))
                lower = item.get('lower', [0,0,0])
                upper = item.get('upper', [0,0,0])
                for sb,v in zip(spinboxes, lower + upper):
                    sb.setValue(int(v))
        except Exception as e:
            QMessageBox.critical(self, 'Error loading file', str(e))

    def save_file(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Save colors', '', 'YAML Files (*.yaml)')
        if not path:
            return
        if not path.lower().endswith(('.yaml','.yml')):
            path += '.yaml'
        try:
            out = []
            for name, spinboxes in self.entries:
                values = [sb.value() for sb in spinboxes]
                out.append({
                    'name':  name.text(),
                    'lower': values[0:3],
                    'upper': values[3:6],
                })
            yaml.dump(out, open(path,'w'))
        except Exception as e:
            QMessageBox.critical(self, 'Error saving file', str(e))

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    tuner = ColorTuner()
    sys.exit(app.exec_())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
