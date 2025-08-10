import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import "qrc:/screens"

ApplicationWindow {
    id: mainWindow
    visible: true
    title: qsTr("MCU")
    height: 860 * 0.85
    width: 1430 * 0.85
    SplitView {
        anchors.margins: 4
        anchors.fill: parent
        orientation: Qt.Vertical
        SplitView {
            orientation: Qt.Horizontal
            width: parent.width
            implicitHeight: mainWindow.height * 0.65
            Rectangle {
                border.width: 1
                border.color: "lightgrey"
                implicitWidth: parent.width / 2
                Trace{
                    manager: serialManager
                }
            }
            Rectangle {
                border.width: 1
                border.color: "lightgrey"

                implicitHeight: mainWindow.height * 0.40
                Trace{
                    manager: netManager
                }
            }
        }
        Rectangle {
            border.width: 1
            border.color: "lightgrey"
            Control{}
        }
    }
}