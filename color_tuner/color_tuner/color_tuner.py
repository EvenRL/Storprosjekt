import sys, yaml
import rclpy
from rclpy.node import Node
from cube_interfaces.msg import ColorRange, ColorRangeArray

from python_qt_binding.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QLineEdit, QFileDialog,
    QMessageBox, QScrollArea, QSizePolicy
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

        self.entries = []  # list of (name_edit, [6 sliders])
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
        name.setFixedWidth(80)
        row.addWidget(name)

        sliders = []
        for i, label in enumerate(['Hl','Sl','Vl','Hu','Su','Vu']):
            col = QVBoxLayout()
            col.addWidget(QLabel(label))
            s = QSlider(Qt.Horizontal)
            s.setRange(0, 179 if i%3==0 else 255)
            s.setValue(0 if i<3 else s.maximum())
            s.sliderReleased.connect(self.publish_all)
            col.addWidget(s)
            row.addLayout(col)
            sliders.append(s)

        self.container_layout.addLayout(row)
        self.entries.append((name, sliders))
        self.publish_all()

    def remove_color(self):
        if not self.entries:
            return
        # remove last
        for i in reversed(range(self.container_layout.count())):
            item = self.container_layout.itemAt(i)
            if i == self.container_layout.count()-1:
                # take and delete all child widgets/layouts
                self.clear_layout(item.layout())
                self.container_layout.takeAt(i)
                break
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
        for name, sliders in self.entries:
            cr = ColorRange()
            cr.name = name.text()
            cr.lower = [int(sliders[i].value()) for i in range(3)]
            cr.upper = [int(sliders[i].value()) for i in range(3,6)]
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
                name, sliders = self.entries[-1]
                name.setText(item['name'])
                for i, v in enumerate(item['lower']+item['upper']):
                    sliders[i].setValue(v)
        except Exception as e:
            QMessageBox.critical(self, 'Error', str(e))

    def save_file(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Save colors', '', 'YAML Files (*.yaml)')
        if not path:
            return
        if not path.lower().endswith(('.yaml','.yml')):
            path += '.yaml'
        try:
            out = []
            for name, sliders in self.entries:
                out.append({
                    'name':  name.text(),
                    'lower': [s.value() for s in sliders[:3]],
                    'upper': [s.value() for s in sliders[3:]],
                })
            yaml.dump(out, open(path,'w'))
        except Exception as e:
            QMessageBox.critical(self, 'Error', str(e))

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    tuner = ColorTuner()
    sys.exit(app.exec_())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
