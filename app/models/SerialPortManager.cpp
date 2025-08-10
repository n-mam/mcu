#include <QDebug>
#include <QTimer>

#include <npb/nm.h>
#include <mcl/log.h>

#include "SerialPortManager.h"

SerialPortManager::SerialPortManager(QObject *parent) : ServerManager(parent) {
    setDecode(false);
}

SerialPortManager::~SerialPortManager() {
    disconnect();
}

void SerialPortManager::connectToPort(QString port) {
    _port = new QSerialPort(port, this);
    _port->setDataBits(QSerialPort::Data8);
    _port->setParity(QSerialPort::NoParity);
    _port->setStopBits(QSerialPort::OneStop);
    _port->setBaudRate(QSerialPort::Baud115200);
    _port->setFlowControl(QSerialPort::NoFlowControl);
    connect(_port, &QSerialPort::readyRead, this, &SerialPortManager::onReadyRead);
    connect(_port, &QSerialPort::errorOccurred, this, &SerialPortManager::onErrorOccurred);
    if (_port->open(QIODevice::ReadWrite)) {
        emit linkStatus(true);
        _port->setDataTerminalReady(true);
        qDebug() << "serial port opened successfully.";
    } else {
        qDebug() << "failed to open serial port:" << _port->errorString();
    }
}

void SerialPortManager::disconnect() {
    if (_port && _port->isOpen()) {
        _port->clear();
        _port->close();
        emit linkStatus(false);
        _port->deleteLater();
        _port = nullptr;
    }
}

void SerialPortManager::transportSink(const std::string& log) {
    if (_port) {
        _port->write((log + "xxx").c_str(), (log + "xxx").size());
        _port->waitForBytesWritten(-1);
    } else {
        LOG << "serial port not connected";
    }
}

void SerialPortManager::onReadyRead() {
    QByteArray buffer = _port->readAll();
    if (buffer.isEmpty()) return;
    ServerManager::processData(buffer);
}

void SerialPortManager::onErrorOccurred(QSerialPort::SerialPortError error) {
    if (error == QSerialPort::ResourceError) {
        qDebug() << "Critical error occurred:" << _port->errorString();
        _port->close();
    }
}
