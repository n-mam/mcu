#include <QDebug>
#include <QVariant>

#include <npb/nm.h>
#include <mcl/log.h>

#include <UdpServerManager.h>

UdpServerManager::UdpServerManager(QObject *parent) : ServerManager(parent) {
    udpSocket = new QUdpSocket(this);
    if (!udpSocket->bind(QHostAddress::Any, 4444, QUdpSocket::ShareAddress)) {
        qDebug() << "Failed to bind to port 4444!";
    } else {
        qDebug() << "listening on port 4444 for UDP broadcast messages";
        udpSocket->setSocketOption(QAbstractSocket::LowDelayOption, QVariant(1));
        udpSocket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, QVariant(8192));
        connect(udpSocket, &QUdpSocket::readyRead, this, &UdpServerManager::processPendingDatagrams);
    }
    setDecode(true);
}

UdpServerManager::~UdpServerManager() {
    delete udpSocket;
}

void UdpServerManager::transportSink(const std::string& log) {
    auto peer = getInstance<config>()->peer_ip;
    if (!peer.size()) {
        LOG << "peer ip not received";
    } else {
        unicastMessage(log, peer, 4445);
    }
}

void UdpServerManager::processPendingDatagrams() {
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray buffer;
        quint16 senderPort;
        QHostAddress senderIP;
        buffer.resize(int(udpSocket->pendingDatagramSize()));
        udpSocket->readDatagram(buffer.data(), buffer.size(), &senderIP, &senderPort);
        ServerManager::processData(buffer);
    }
    if (getInstance<config>()->peer_ip.size()) {
        emit linkStatus(true);
    }
}

void UdpServerManager::unicastMessage(const std::string& data, const std::string& ip, quint16 port) {
    auto rc = udpSocket->writeDatagram(
        data.c_str(), data.size(), QHostAddress(QString::fromStdString(ip)), port);
    if (rc == -1) {
        LOG << "unicastMessage: " << rc << ", "
                << udpSocket->errorString().toStdString();
    }
    udpSocket->flush();
    udpSocket->waitForBytesWritten();
}