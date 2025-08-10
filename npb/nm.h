#ifndef NANOMSG_H
#define NANOMSG_H

#include <any>
#include <utility>

#include <mcu.pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include <mcl/config.h>

namespace nanomsg {

inline auto write_nano_msg_a(const char *data, mcl::log::sink sink, void *target = nullptr) {
    size_t en_len = 0;
    uint8_t en_buf[512];
    Packet pkt = Packet_init_zero;
    pkt.which_payload = Packet_msg_a_tag;
    snprintf(pkt.payload.msg_a.data, sizeof(pkt.payload.msg_a.data), "%s", data);
    pb_ostream_t stream = pb_ostream_from_buffer(en_buf, sizeof(en_buf));
    if (!pb_encode_delimited(&stream, Packet_fields, &pkt)) {
        std::cout << "encoding failed: " << PB_GET_ERROR(&stream) << std::endl;
    } else {
        en_len = stream.bytes_written;
        std::string nano_msg(reinterpret_cast<char*>(en_buf), en_len);
        PBMSG(sink, target) << nano_msg;
    }
}

inline auto write_nano_msg_b(int id, mcl::log::sink sink, void *target = nullptr) {
    size_t en_len = 0;
    uint8_t en_buf[512];
    Packet pkt = Packet_init_zero;
    pkt.which_payload = Packet_msg_b_tag;
    pkt.payload.msg_b.id = id;
    pb_ostream_t stream = pb_ostream_from_buffer(en_buf, sizeof(en_buf));
    if (!pb_encode_delimited(&stream, Packet_fields, &pkt)) {
        std::cout << "encoding failed: " << PB_GET_ERROR(&stream) << std::endl;
    }
    en_len = stream.bytes_written;
    std::string nano_msg(reinterpret_cast<char *>(en_buf), en_len);
    PBMSG(sink, target) << nano_msg;
}

inline auto write_nano_msg_c(bool status, mcl::log::sink sink, void *target = nullptr) {
    size_t en_len = 0;
    uint8_t en_buf[512];
    Packet pkt = Packet_init_zero;
    pkt.which_payload = Packet_msg_c_tag;
    pkt.payload.msg_c.status = status;
    pb_ostream_t stream = pb_ostream_from_buffer(en_buf, sizeof(en_buf));
    if (!pb_encode_delimited(&stream, Packet_fields, &pkt)) {
        std::cout << "encoding failed: " << PB_GET_ERROR(&stream) << std::endl;
    }
    en_len = stream.bytes_written;
    std::string nano_msg(reinterpret_cast<char *>(en_buf), en_len);
    PBMSG(sink, target) << nano_msg;
}

inline void write_nano_msg_discovery(const std::string& data, mcl::log::sink sink, void *target = nullptr) {
    size_t en_len = 0;
    uint8_t en_buf[512];
    Packet pkt = Packet_init_zero;
    pkt.which_payload = Packet_msg_d_tag;
    snprintf(pkt.payload.msg_d.ip, sizeof(pkt.payload.msg_a.data), "%s", data.c_str());
    pb_ostream_t stream = pb_ostream_from_buffer(en_buf, sizeof(en_buf));
    if (!pb_encode_delimited(&stream, Packet_fields, &pkt)) {
        std::cout << "encoding failed: " << PB_GET_ERROR(&stream) << std::endl;
    } else {
        en_len = stream.bytes_written;
        std::string nano_msg(reinterpret_cast<char *>(en_buf), en_len);
        PBMSG(sink, target) << nano_msg;
    }
}

inline auto write_nano_msg_kv(const KeyValue& kv, mcl::log::sink sink, void *target = nullptr) {
    //std::cout << "sending Key : " << kv.key << ", Value : " << kv.value << std::endl;
    Packet pkt = Packet_init_zero;
    pkt.payload.msg_kv.key = kv.key;
    pkt.payload.msg_kv.value = kv.value;
    pkt.which_payload = Packet_msg_kv_tag;
    uint8_t en_buf[512];
    pb_ostream_t stream = pb_ostream_from_buffer(en_buf, sizeof(en_buf));
    if (!pb_encode_delimited(&stream, Packet_fields, &pkt)) {
        std::cerr << "encoding failed: " << PB_GET_ERROR(&stream) << std::endl;
        return;
    }
    size_t en_len = stream.bytes_written;
    std::string nano_msg(reinterpret_cast<char *>(en_buf), en_len);
    PBMSG(sink, target) << nano_msg;
}

inline auto write_nano_msg_log(const char *data, mcl::log::sink sink, void *target = nullptr) {
    size_t en_len = 0;
    uint8_t en_buf[512];
    Packet pkt = Packet_init_zero;
    pkt.which_payload = Packet_msg_log_tag;
    snprintf(pkt.payload.msg_a.data, sizeof(pkt.payload.msg_a.data), "%s", data);
    pb_ostream_t stream = pb_ostream_from_buffer(en_buf, sizeof(en_buf));
    if (!pb_encode_delimited(&stream, Packet_fields, &pkt)) {
        std::cout << "encoding failed: " << PB_GET_ERROR(&stream) << std::endl;
    } else {
        en_len = stream.bytes_written;
        std::string nano_msg(reinterpret_cast<char *>(en_buf), en_len);
        PBMSG(sink, target) << nano_msg;
    }
}

using TDecodeCallback = std::function<void (std::pair<int, std::any>)>;

inline auto decode(uint8_t* buf, int len, TDecodeCallback callback = nullptr) {
    bool fRet = false;
    Packet pkt = Packet_init_zero;
    std::pair<int, std::any> result;
    pb_istream_t stream = pb_istream_from_buffer(buf, len);
    while (stream.bytes_left) {
        auto success = fRet = pb_decode_delimited(&stream, Packet_fields, &pkt);
        if (success) {
            switch (pkt.which_payload) {
                case Packet_msg_a_tag:
                    LOG << "Message_A: " << pkt.payload.msg_a.data;
                    result = std::make_pair(Packet_msg_a_tag, std::string(pkt.payload.msg_a.data));
                    break;
                case Packet_msg_b_tag:
                    LOG << "Message_B: " << std::dec << pkt.payload.msg_b.id;
                    result = std::make_pair(Packet_msg_b_tag, pkt.payload.msg_b.id);
                    break;
                case Packet_msg_c_tag:
                    LOG << "Message_C: " << pkt.payload.msg_c.status;
                    result = std::make_pair(Packet_msg_c_tag, pkt.payload.msg_c.status);
                    break;
                case Packet_msg_d_tag: {
                    LOG << "Discovery: " << pkt.payload.msg_d.ip ;
                    getInstance<config>()->peer_ip = std::string(pkt.payload.msg_d.ip);
                    result = std::make_pair(Packet_msg_d_tag, std::string(pkt.payload.msg_d.ip));
                    break;
                }
                case Packet_msg_kv_tag:
                    getInstance<config>()->setKeyValue(pkt.payload.msg_kv);
                    break;
                case Packet_msg_log_tag:
                    result = std::make_pair(Packet_msg_log_tag, std::string(pkt.payload.msg_log.data));
                    break;
                default:
                    LOG << "invalid nanopb message type";
                    LOG << mcl::dumpBuffer(buf, len);
                    break;
            }
        } else {
            LOG << "failed to decode packet: " << PB_GET_ERROR(&stream);
            LOG << mcl::dumpBuffer(buf, len);
        }
    }
    if (callback) callback(result);
    return fRet;
}

}

#endif