import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Item {
    anchors.margins: 6
    anchors.fill: parent
    property var manager: serialManager
    Column {
        Row {
            spacing: 4
            TextField {
                id: port
                height: 36
                width: 100
                placeholderText: "COM"
                anchors.verticalCenter: parent.verticalCenter
            }
            Button {
                text: "Connect"
                onClicked: serialManager.connectToPort("COM" + port.text)
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
            Button {
                text: "Disconnect"
                onClicked: serialManager.disconnect();
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
        }
        RowLayout {
            RadioButton {
                id: serial
                checked: true
                text: qsTr("Serial")
                onCheckedChanged: {
                    if (checked) {
                        manager = serialManager
                    }
                }
            }
            RadioButton {
                text: qsTr("Network")
                onCheckedChanged: {
                    manager = netManager
                }
            }
        }
        Row {
            spacing: 4
            TextField {
                id: message
                height: 36
                width: 245
                placeholderText: qsTr("Message")
                anchors.verticalCenter: parent.verticalCenter
            }
            Button {
                text: "Send"
                onClicked: manager.sendMessage(message.text)
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
        }
        Row {
            spacing: 4
            TextField {
                id: key
                height: 36
                placeholderText: qsTr("Key")
                anchors.verticalCenter: parent.verticalCenter
            }
            TextField {
                id: value
                height: 36
                placeholderText: qsTr("Value")
                anchors.verticalCenter: parent.verticalCenter
            }
            Button {
                text: "Send"
                onClicked: manager.sendKeyValue(parseInt(key.text), parseFloat(value.text))
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
        }
        Row {
            spacing: 4
            Button {
                text: "Stop"
                onClicked: manager.stopWorkflowLoop();
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
            Button {
                text: "Reset"
                onClicked: manager.resetMCU();
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
        }
    }
}