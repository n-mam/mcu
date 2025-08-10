import QtQuick
import QtQuick.Controls

Item {
    width: parent.width
    height: parent.height
    required property var manager;
    Rectangle {
        id: status
        width: 20
        height: 12
        radius: 2
        border.width: 1
        border.color: "grey"
        anchors.top: parent.top
        anchors.topMargin: 4
        anchors.rightMargin: 10
        property bool active: false
        anchors.right: parent.right
        color: (status.active ? "lawngreen" : "tomato")
        Connections {
            target: manager
            function onLinkStatus(on) {
                status.active = on;
            }
        }
    }
    Column {
        anchors.fill: parent
        ListModel {
            id: traceModel
            ListElement { line: "" }
        }
        ListView {
            id: traceList
            clip: true
            leftMargin: 8
            rightMargin: 8
            model: traceModel
            width: parent.width
            height: parent.height * 0.90
            ScrollBar.vertical: ScrollBar {}
            flickableDirection: Flickable.VerticalFlick
            delegate: Rectangle {
                height: 17
                Label {
                    text: line
                    textFormat: Text.PlainText
                }
            }
            Connections {
                target: manager
                enabled: (traceEnable.checkState === Qt.Checked)
                function onQmlLog(log) {
                    for (var x of log.split("\n")) {
                        traceModel.append({
                            line: new Date().toLocaleTimeString(Qt.locale(),
                            "hh:" + "mm:" + "ss:" + "zzz") + " " + x
                        })
                    }
                    traceList.positionViewAtEnd()
                }
            }
        }
        Row {
            id: logActions
            spacing: 5
            height: parent.height * 0.10
            anchors.horizontalCenter: parent.horizontalCenter
            Button {
                id: clearButton
                text: "Clear"
                onClicked: traceModel.clear()
                anchors.verticalCenter: parent.verticalCenter
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
            Button {
                id: savebutton
                text: "Save"
                onClicked: {}
                anchors.verticalCenter: parent.verticalCenter
                background: Rectangle {
                    radius: 4
                    border.width: 1
                    border.color: "grey"
                    color: "lightgrey"
                }
            }
            CheckBox {
                id: traceEnable
                z: 2
                checked: true
                anchors.topMargin: 3
                text: qsTr("enable")
                anchors.verticalCenter: parent.verticalCenter
            }
            CheckBox {
                id: nanopb
                z: 2
                checked: manager.decode
                anchors.topMargin: 3
                text: qsTr("nanopb")
                anchors.verticalCenter: parent.verticalCenter
                onCheckedChanged: {
                    manager.decode = checked
                }
            }
        }
    }
    onVisibleChanged: {
        if (visible) {
          traceList.positionViewAtEnd()
        }
    }
}