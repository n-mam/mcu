#include <QDebug>
#include <QTimer>

#include <npb/nm.h>
#include <mcl/log.h>

#include <TcpServerManager.h>

TcpServerManager::TcpServerManager(QObject *parent) : ServerManager(parent), _socket(this) {
    connect(&_socket, &QTcpSocket::connected, this, &TcpServerManager::onConnected);
    connect(&_socket, &QTcpSocket::readyRead, this, &TcpServerManager::onReadyRead);
    connect(&_socket, &QTcpSocket::disconnected, this, &TcpServerManager::onDisconnected);
    connect(&_socket, &QTcpSocket::errorOccurred, this, &TcpServerManager::onErrorOccurred);
    tcpHostConnect(_host,_port);
    setDecode(true);
}

TcpServerManager::~TcpServerManager() {
    if (_socket.isOpen()) {
        _socket.close();
    }
}

void TcpServerManager::tcpHostConnect(QString host, int port) {
    _host = host;
    _port = port;
    if (_socket.state() == QTcpSocket::ConnectedState) {
        _socket.disconnectFromHost();
    }
    _socket.connectToHost(_host, _port);
    qDebug()<<"Ip is:- "<<host<<"port is:- "<<port;
}

void TcpServerManager::transportSink(const std::string& log) {
    _socket.write(log.c_str(), log.size());
}

void TcpServerManager::onConnected() {
    emit linkStatus(true);
    qDebug() << "connected to " << _host << ":" << _port;
}

void TcpServerManager::onDisconnected() {
    emit linkStatus(false);
    qDebug() << "disconnected from " << _host << ":" << _port;
}

void TcpServerManager::onReadyRead() {
    QByteArray buffer = _socket.readAll();
    if (buffer.isEmpty()) return;
    ServerManager::processData(buffer);
}

void TcpServerManager::onErrorOccurred(QAbstractSocket::SocketError error) {
    if (error == QAbstractSocket::RemoteHostClosedError) {
        qDebug() << "Remote host closed the connection.";
    } else {
        qDebug() << "An error occurred:" << _socket.errorString();
        _socket.close();
    }
}
