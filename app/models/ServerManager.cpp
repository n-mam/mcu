#include <ServerManager.h>

#include <QDebug>
#include <QNetworkInterface>

#include <npb/nm.h>

ServerManager::ServerManager(QObject *parent) : QObject(parent) {
    mcl::log::setLogLevel(mcl::log::debug);
    mcl::log::setLogSink<std::string>(
        [this](int level, int sink, auto log) {
            if (sink == mcl::log::sink::net) {
                transportSink(log);
            } else if (sink == mcl::log::sink::con) {
                logToQml(log);
            }
        }, this);
}

ServerManager::~ServerManager() {
    mcl::log::setLogSink<std::string>(nullptr);
}

void ServerManager::sendMessage(QString message) {
    // nanomsg::write_nano_msg_b(123);
    // nanomsg::write_nano_msg_c(true);
    nanomsg::write_nano_msg_a(message.toStdString().c_str(), mcl::log::sink::net, this);
}

void ServerManager::sendKeyValue(uint32_t key, float value) {
    nanomsg::write_nano_msg_kv({key, value}, mcl::log::sink::net, this);
}

void ServerManager::resetMCU() {
    nanomsg::write_nano_msg_kv({config::key::action, 99}, mcl::log::sink::net, this);
}

void ServerManager::stopWorkflowLoop() {
    nanomsg::write_nano_msg_kv({config::key::action, 0}, mcl::log::sink::net, this);
}

void ServerManager::processData(QByteArray& buffer) {
    if (m_decode) {
        nanomsg::decode(reinterpret_cast<uint8_t *>(buffer.data()), buffer.size(),
            [this](const std::pair<int, std::any>& result) {
                if (result.second.has_value()) {
                    std::stringstream ss;
                    if (result.first == Packet_msg_a_tag) {
                        ss << "Message_A: " << std::any_cast<std::string>(result.second);
                    } else if (result.first == Packet_msg_b_tag) {
                        ss << "Message_B: " << std::any_cast<int32_t>(result.second);
                    } else if (result.first == Packet_msg_c_tag) {
                        ss << "Message_C: " << std::any_cast<bool>(result.second);
                    } else if (result.first == Packet_msg_d_tag) {
                        ss << "Discovery: " << std::any_cast<std::string>(result.second);
                    } else if (result.first == Packet_msg_log_tag) {
                        ss << std::any_cast<std::string>(result.second);
                    } else if (0) {

                    }
                    logToQml(ss.str());
                }
            });
    } else {
        logToQml(buffer.toStdString());
        //LOG << mcl::dumpBuffer(reinterpret_cast<uint8_t *>(buffer.data()), buffer.size());
    }
}

void ServerManager::logToQml(const std::string& log) {
    emit qmlLog(QString::fromStdString(log));
}

QString ServerManager::getLocalIPAddress() {
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface &interface : interfaces) {
        if (interface.flags() & QNetworkInterface::IsLoopBack) continue;
        QList<QNetworkAddressEntry> entries = interface.addressEntries();
        for (const auto &entry : entries) {
            QHostAddress address = entry.ip();
            //qDebug()  << "ip " << address.toString();
            if (!address.isLoopback() &&
                address.toString().startsWith("192.168.") &&
                address.protocol() == QAbstractSocket::IPv4Protocol) {
                    qDebug() << "local ip: " << address.toString();
                    return address.toString();
            }
        }
    }
    return "";
}

bool ServerManager::getDecode() {
    return m_decode;
}

void ServerManager::setDecode(bool decode) {
    if (decode != m_decode) {
        m_decode = decode;
        emit decodeChanged();
    }
}
