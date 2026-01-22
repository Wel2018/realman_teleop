from toolbox.qt import qtbase


def set_layout_visible(layout: qtbase.QLayout, visible: bool):
    """设置指定布局下所有组件的可见性"""
    for i in range(layout.count()):
        item = layout.itemAt(i)
        if item.widget(): # type: ignore
            item.widget().setVisible(visible) # type: ignore

        elif item.layout(): # type: ignore
            set_layout_visible(item.layout(), visible) # type: ignore


def set_layout_enabled(layout: qtbase.QLayout, enable: bool):
    """设置指定布局下所有组件的可用性"""
    for i in range(layout.count()):
        item = layout.itemAt(i)
        if item.widget(): # type: ignore
            item.widget().setEnabled(enable) # type: ignore

        elif item.layout(): # type: ignore
            set_layout_enabled(item.layout(), enable) # type: ignore
