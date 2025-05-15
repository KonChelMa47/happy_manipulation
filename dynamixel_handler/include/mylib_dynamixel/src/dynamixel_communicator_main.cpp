#include "dynamixel_communicator.h"
#include <stdio.h>

// #define DXL_LOBYTE(w) ((uint8_t)(((uint64_t)(w)) & 0xff))
// #define DXL_HIBYTE(w) ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

static const uint8_t INSTRUCTION_PING          = 0x01;
static const uint8_t INSTRUCTION_READ          = 0x02;
static const uint8_t INSTRUCTION_WRITE         = 0x03;
static const uint8_t INSTRUCTION_RED_WRITE     = 0x04;
static const uint8_t INSTRUCTION_ACTION        = 0x05;
static const uint8_t INSTRUCTION_FACTORY_RESET = 0x06;
static const uint8_t INSTRUCTION_REBOOT        = 0x08;
static const uint8_t INSTRUCTION_SYNC_READ     = 0x82;
static const uint8_t INSTRUCTION_SYNC_WRITE    = 0x83;
static const uint8_t INSTRUCTION_FAST_SYNC_READ= 0x8A;
static const uint8_t INSTRUCTION_BULK_READ     = 0x92;
static const uint8_t INSTRUCTION_BULK_WRITE    = 0x93;
static const uint8_t INSTRUCTION_FAST_BULK_READ= 0x9A;

static const uint8_t HEADER[4] = {0xFF, 0xFF, 0xFD, 0x00};

static const uint8_t COM_ERROR_RESULT_FAIL = 0;
static const uint8_t COM_ERROR_INSTRUCTION = 1;
static const uint8_t COM_ERROR_CRC         = 2;
static const uint8_t COM_ERROR_DATA_RANGE  = 3;
static const uint8_t COM_ERROR_DATA_LENGTH = 4;
static const uint8_t COM_ERROR_DATA_LIMIT  = 5;
static const uint8_t COM_ERROR_ACCESS      = 6;
static const uint8_t COM_ERROR_ALERT       = 7;

uint16_t DynamixelCommunicator::CalcChecksum(uint8_t data[], uint8_t length) {
  uint16_t crc_accum = 0;
  uint16_t i;
  uint16_t crc_table[256] = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

  for (uint16_t j = 0; j < length; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ data[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

/** @fn
 * @brief 送信用にdata_write_[]にデータを格納する
 * @param DynamixelDataType type 変換後のデータ型
 * @param int64_t data_int 書き込む値．int型に変換済み．どの型にも対応できるようにint64_t
 * @return void
 */
void DynamixelCommunicator::EncodeDataWrite(DynamixelDataType type, int64_t data_int) {
  switch (type) {
    case TYPE_INT8 : {
      int8_t d = int8_t(data_int);
      data_write_[0] = d & 0xFF;
      break;
    }
    case TYPE_UINT8 : {
      uint8_t d = uint8_t(data_int);
      data_write_[0] = d & 0xFF;
      break;
    }
    case TYPE_INT16 : {
      int16_t d = int16_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      break;
    }
    case TYPE_UINT16 : {
      uint16_t d = uint16_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      break;
    }
    case TYPE_INT32 : {
      int32_t d = int32_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      data_write_[2] = (d>>16) & 0xFF;
      data_write_[3] = (d>>24) & 0xFF;
      break;
    }
    case TYPE_UINT32 : {
      uint32_t d = uint32_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      data_write_[2] = (d>>16) & 0xFF;
      data_write_[3] = (d>>24) & 0xFF;
      break;
    }
  }
}

/** @fn
 * @brief data_read_[]に格納されたデータをつなげて一つのデータに復元する
 * @param DynamixelDataType type 変換後のデータ型
 * @return (int64_t) 変換後の値．どの型にも対応できるようにint64_t
 */
int64_t DynamixelCommunicator::DecodeDataRead(DynamixelDataType type) {
  switch (type) {
    case TYPE_INT8 : {
      int8_t data_int = 0;
      data_int |= data_read_[0] << 0;
      return int64_t(data_int);
      break;
    }
    case TYPE_UINT8 : {
      uint8_t data_int = 0;
      data_int |= data_read_[0] << 0;
      return int64_t(data_int);
      break;
    }
    case TYPE_INT16 : {
      int16_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      return int64_t(data_int);
      break;
    }
    case TYPE_UINT16 :  {
      uint16_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      return int64_t(data_int);
      break;
    }
    case TYPE_INT32 : {
      int32_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      data_int |= data_read_[2] << 16;
      data_int |= data_read_[3] << 24;
      return int64_t(data_int);
      break;
    }
    case TYPE_UINT32 : {
      uint32_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      data_int |= data_read_[2] << 16;
      data_int |= data_read_[3] << 24;
      return int64_t(data_int);
      break;
    }
  }
  return 0;
}

/** @fn
 * @brief port_handler_を用いて，ポートを開く
 */
bool DynamixelCommunicator::OpenPort() {
  if (port_handler_->openPort()) {
    if (verbose_) printf("Succeeded to open the port : %s!\n", port_name_);
  } else {
    printf("Failed to open the port : %s!\n", port_name_);
    return false;
  }

  port_handler_->setLatencyTimer(latency_timer_);
  if (verbose_) printf("Succeeded to change the latency timer : %d!\n", latency_timer_);

  if (port_handler_->setBaudRate(baudrate_)) {
    if (verbose_) printf("Succeeded to change the baudrate : %d!\n", baudrate_);
    return true;
  } else {
    printf("Failed to change the baudrate : %d!\n", baudrate_);
    return false;
  }
}

/** @fn
 * @brief port_handler_を用いて，ポートを閉じる
 */
bool DynamixelCommunicator::ClosePort() {
  port_handler_->closePort();
  if (verbose_) printf("Close port : %s!\n", port_name_);
  return true;
}

/** @fn
 * @brief 指定したDynamixelをリブートする
 * @param uint8_t servo_id 対象のID
 */
void DynamixelCommunicator::Reboot(uint8_t servo_id) {
  uint8_t send_data[10] = {0};
  uint16_t length = 3;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_REBOOT;  // instruction
  uint16_t sum = CalcChecksum(send_data, 8);
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  port_handler_->clearPort();
  port_handler_->writePort(send_data, 10);

  if (status_return_level_ != 2) return;
 
  timeout_last_read_ = false;
  port_handler_->setPacketTimeout( uint16_t(11) );
  while(port_handler_->getBytesAvailable() < 11) {
    if (port_handler_->isPacketTimeout()) {
      if(verbose_) printf("Reboot Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
      timeout_last_read_ = true;
      return;
    }
  }

  uint8_t read_data[11];
  port_handler_->readPort(read_data, 11);
}

/** @fn
 * @brief 指定したDynamixelをファクトリーリセットする
 * @param uint8_t servo_id 対象のID
 * @param FactoryResetLevel level リセットレベル
 */
void DynamixelCommunicator::FactoryReset(uint8_t servo_id, FactoryResetLevel level) {
  uint8_t send_data[11] = {0};
  uint16_t length = 4;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_FACTORY_RESET;  // instruction
  send_data[8] = level;
  uint16_t sum = CalcChecksum(send_data, 9);
  send_data[9] = sum & 0xFF;
  send_data[10] = (sum>>8) & 0xFF;
  port_handler_->writePort(send_data, 11);

  if (status_return_level_ != 2) return; 

  uint8_t read_data[11];
  port_handler_->readPort(read_data, 11);
}


/** @fn
 * @brief 指定したDynamixelにpingを送り，応答があるかをboolで返す
 * @param uint8_t servo_id 対象のID
 * @return bool 応答があったかどうか
 */
bool DynamixelCommunicator::Ping(uint8_t servo_id) {
  uint8_t send_data[10] = {0};
  uint16_t length = 3;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_PING;  // instruction
  uint16_t sum = CalcChecksum(send_data, 8);
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  port_handler_->clearPort();
  port_handler_->writePort(send_data, 10);

  port_handler_->setPacketTimeout( uint16_t(14) );
  while(port_handler_->getBytesAvailable() < 14) 
    if (port_handler_->isPacketTimeout()) {
        if(verbose_) printf("Ping Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
        return false;
    }

  uint8_t read_data[14];
  uint8_t read_length = port_handler_->readPort(read_data, 14);
    if (read_length != 14) {
        if(verbose_) printf("Ping Error(no data): ID %d, data_length = %d\n", servo_id, read_length);
        return false;
    }
  if( read_data[4] == servo_id) {
    return true;
  } else {
    if (verbose_) printf("Ping Error(header or id): expected ID %d, return ID %d\n", servo_id, read_data[4]);
    return false;
  }
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param uint8_t servo_id 対象のID
 * @param int64_t data_int 書き込むデータ．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::Write(DynamixelAddress dp, uint8_t servo_id, int64_t data_int) {
    uint8_t send_data[16] = {0}; // 書き込むdpのサイズによって変わるが， dp.size()は最大4 4+12=16より十分
    uint16_t length = dp.size()+5;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_WRITE;  // instruction
    send_data[8] = dp.address() & 0xFF;
    send_data[9] = (dp.address()>>8) & 0xFF;
    EncodeDataWrite(dp.data_type(), data_int);
    for(size_t i=0; i<dp.size(); i++) {
        send_data[10+i] = data_write_[i];
    }
    uint16_t sum = CalcChecksum(send_data, 10+dp.size());
    send_data[10+dp.size()] = sum & 0xFF;
    send_data[11+dp.size()] = (sum>>8) & 0xFF;

    port_handler_->clearPort();
    port_handler_->writePort(send_data, 12+dp.size());

    if (status_return_level_ != 2) return true;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout( uint16_t(11) );
    while(port_handler_->getBytesAvailable() < 11) {
        if (port_handler_->isPacketTimeout()) {
        if(verbose_) printf("Write Error(return status time out): ID %d, available bytes %d/11\n", (int)servo_id, port_handler_->getBytesAvailable());
        timeout_last_read_ = true;
        return false;
        }
    }
    uint8_t read_data[11]; // Writeのステータスパケットは固定で11
    port_handler_->readPort(read_data, 11);
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
        if(verbose_) printf("Write Error(return status header or id): ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return false;
    }
    uint8_t error = (uint8_t)read_data[8];
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Write Error(return status packet error) : ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return false;
    } 
    uint16_t sum_est = CalcChecksum(read_data, 9);
    uint16_t sum_read = uint16_t(read_data[9]) | uint16_t(read_data[9+1])<<8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Write Error(return status crc): ID %d, est=%d, read=%d\n", (int)servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return false;
    }
    // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    return true;
}

/** @fn
 * @brief Dynamixelから情報を読み込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param uint8_t servo_id 対象のID
 * @return (int64_t) 読み込んだデータ．intのまま
 */
int64_t DynamixelCommunicator::Read(DynamixelAddress dp, uint8_t servo_id) {
    uint8_t send_data[14] = {0}; // Readのインストラクションパケットは固定で14
    uint16_t length = 7;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_READ;  // instruction
    send_data[8] = dp.address() & 0xFF;
    send_data[9] = (dp.address()>>8) & 0xFF;
    send_data[10] = dp.size() & 0xFF;
    send_data[11] = (dp.size()>>8) & 0xFF;
    uint16_t sum = CalcChecksum(send_data, 12);
    send_data[12] = sum & 0xFF;
    send_data[13] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Read Warn: clear buffer time out\n");
            break;
        }
    }
    port_handler_->clearPort();
    port_handler_->writePort(send_data, 14);

    if (status_return_level_ == 0) return 0;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout( uint16_t(11+dp.size()) );
    while(port_handler_->getBytesAvailable() < 11+dp.size()) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Read Error(time out): ID %d, available bytes %d / %d\n", (int)servo_id, port_handler_->getBytesAvailable(), (int)(11+dp.size()));
            deficient_byte_ = 11+dp.size() - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return 0;
        }
    }

    uint8_t read_data[15]; // 11+dp.size()<=11+4=15より十分
    uint8_t read_length = port_handler_->readPort(read_data, 11+dp.size());
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_length != 11+dp.size()) {
        if(verbose_) printf("Read Error(no data): ID %d, data_length = %d\n", (int)servo_id, read_length);
        comm_error_last_read_ = true;
        return 0;
    }
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
        if(verbose_) printf("Read Error(header or id): ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return 0;
    }
    uint8_t error = (uint8_t)read_data[8];
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Read Error(packet error) : ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return 0;
    } 
    uint16_t sum_est = CalcChecksum(read_data, 9+dp.size());
    uint16_t sum_read = uint16_t(read_data[9+dp.size()]) | uint16_t(read_data[9+dp.size()+1])<<8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Read Error(crc): ID %d, est=%d, read=%d\n", (int)servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return 0;
    }
    // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[9+i];
    return DecodeDataRead(dp.data_type());
}


/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報を書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param vector<uint8_t> servo_id_list servo_id_list 書き込むサーボのIDのベクトル
 * @param vector<uint64_t> data_int_list 書き込むデータのリスト．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(DynamixelAddress dp,  const vector<uint8_t>& servo_id_list, const vector<int64_t>& data_int_list) {
  if (servo_id_list.size() > 100) {
    if(verbose_) printf("Sync Write Error(too many servo): servo num=%d > 100\n", (int)servo_id_list.size());
    return false;
  }
  if (servo_id_list.size() != data_int_list.size()) {
    if(verbose_) printf("Sync Write Error(mismatch servo and data num): servo num=%d, data num=%d\n", (int)servo_id_list.size(), (int)data_int_list.size());
    return false;
  }
  uint8_t send_data[514] = {0}; // 書き込むサーボの数によって変わるが， dp.size()は最大4 (4+1)*100+14=514 100サーボに同時に4バイト書き込むことはないので十分
  int8_t num_servo = servo_id_list.size();
  uint16_t length = (1+dp.size())*num_servo+7;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = 0xFE;  // id
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_SYNC_WRITE;  // instruction
  send_data[8] = dp.address() & 0xFF;
  send_data[9] = (dp.address()>>8) & 0xFF;
  send_data[10] = dp.size() & 0xFF;
  send_data[11] = (dp.size()>>8) & 0xFF;
  for(int i_servo=0; i_servo<num_servo; i_servo++) {
    send_data[12+i_servo*(dp.size()+1)] = servo_id_list[i_servo];
    EncodeDataWrite(dp.data_type(), data_int_list[i_servo]);
    for(int i_data=0; i_data<dp.size(); i_data++) {
      send_data[12+i_servo*(dp.size()+1)+1+i_data] = data_write_[i_data];
    }
  }
  uint16_t sum = CalcChecksum(send_data, 12+num_servo*(dp.size()+1));
  send_data[12+num_servo*(dp.size()+1)] = sum & 0xFF;
  send_data[13+num_servo*(dp.size()+1)] = (sum>>8) & 0xFF;

  port_handler_->clearPort();
  port_handler_->writePort(send_data, 14+num_servo*(dp.size()+1));
  return true;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param map<uint8_t, int64_t> id_data_int_map 書き込むサーボのIDと書き込むデータのマップ
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(DynamixelAddress dp, const map<uint8_t, int64_t>& id_data_int_map) {
    std::vector<uint8_t>  servo_id_list; servo_id_list.reserve(id_data_int_map.size());
    std::vector<int64_t> data_int_list; data_int_list.reserve(id_data_int_map.size());
    for (const auto& id_data : id_data_int_map) {
        servo_id_list.push_back(id_data.first);    // キー（サーボID）を追加
        data_int_list.push_back(id_data.second);   // 値（データ）を追加
    }
    return SyncWrite( dp, servo_id_list, data_int_list );
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, int64_t> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, int64_t> DynamixelCommunicator::SyncRead( DynamixelAddress dp, const vector<uint8_t>& servo_id_list) {
    if (servo_id_list.size() > 100) {
        if(verbose_) printf("Sync Read Error(too many servo): servo num=%d > 100\n", (int)servo_id_list.size());
        return map<uint8_t, int64_t>();
    }
    uint8_t send_data[114] = {0}; // 読み込むサーボの数によって変わるが， 100+14=114 100サーボに同時に読み込むことはないので十分
    uint8_t num_servo = servo_id_list.size();
    uint16_t length = num_servo+7;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = 0xFE;  // id
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_SYNC_READ;  // instruction
    send_data[8] = dp.address() & 0xFF;
    send_data[9] = (dp.address()>>8) & 0xFF;
    send_data[10] = dp.size() & 0xFF;
    send_data[11] = (dp.size()>>8) & 0xFF;
    for(int i_servo=0; i_servo<num_servo; i_servo++) {
        send_data[12+i_servo] = servo_id_list[i_servo];
    }
    uint16_t sum = CalcChecksum(send_data, 12+num_servo);
    send_data[12+num_servo] = sum & 0xFF;
    send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read Warn: clear buffer time out\n");
            break;
        }
    }
    port_handler_->clearPort();
    port_handler_->writePort(send_data, 14+num_servo);

    // データ読み込みの処理
    uint8_t read_data[15]; // 11+dp.size()<=11+4=15より十分 
    map<uint8_t, int64_t> id_data_int_map;
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    comm_error_last_read_ = false;
    for(int i_servo=0; i_servo<num_servo; i_servo++) {
        timeout_last_read_ = false;
        port_handler_->setPacketTimeout( uint16_t(11+dp.size()) );
        while(port_handler_->getBytesAvailable() < 11+dp.size()) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Sync Read Error(time out): ID %d, available bytes %d / %d\n", servo_id_list[i_servo], port_handler_->getBytesAvailable(), (int)(11+dp.size()));
            deficient_byte_ = 11+dp.size() - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return id_data_int_map; // これ以降すべての読み込みを諦める．
        }
        }

        int8_t read_length = port_handler_->readPort(read_data, 11+dp.size());

        //  エラーチェック
        if (read_length == -1) {
            if(verbose_) printf("Sync Read Error(read port) : ID %d\n", servo_id_list[i_servo]);
            comm_error_last_read_ = true;
            continue;
        }
        if (read_data[0] != HEADER[0] or
            read_data[1] != HEADER[1] or
            read_data[2] != HEADER[2] or
            read_data[3] != HEADER[3]) {
            if(verbose_) printf("Sync Read Error(header): ID %d\n", servo_id_list[i_servo]);
            comm_error_last_read_ = true;
            continue;
        }
        uint8_t id = read_data[4];
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Sync Read Error(packet id) : est ID=%d, read ID=%d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            continue;
        }
        uint8_t error = (uint8_t)read_data[8];
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue;
        }
        uint16_t sum_est = CalcChecksum(read_data, 9+dp.size());
        uint16_t sum_read = uint16_t(read_data[9+dp.size()]) | uint16_t(read_data[9+dp.size()+1])<<8;
        if (sum_est != sum_read) {
            if(verbose_) printf("Sync Read Error(crc): ID %d, est=%d, read=%d\n", id, sum_est, sum_read);
            comm_error_last_read_ = true;
            continue;
        }

        // if(verbose_) {printf("read:" ); for (size_t i=0; i<read_length; i++) printf("%02X ", read_data[i]); printf("\n");}

        // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
        if ( error & 0x80 ) hardware_error_last_read_ = true; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if ( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
        for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[9+i];
        id_data_int_map[id] = DecodeDataRead(dp.data_type());
    }
    return id_data_int_map;
}


/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む,パケットを最小限にすることで高速化したもの．サーボ1つでも失敗するとすべて読み込めない
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, int64_t> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, int64_t> DynamixelCommunicator::SyncRead_fast(DynamixelAddress dp, const vector<uint8_t>& servo_id_list) {
    if (servo_id_list.size() > 50) {
        if(verbose_) printf("Fast Sync Read Error(too many servo): servo num=%d > 50\n", (int)servo_id_list.size());
        return map<uint8_t, int64_t>();
    }
	uint8_t send_data[64] = {0}; // 読み込むサーボの数によって変わるが， 50+14=64 50サーボに同時に読み込むことはないので十分
	uint8_t num_servo = servo_id_list.size();
	uint16_t length = num_servo+7;
	send_data[0] = HEADER[0];
	send_data[1] = HEADER[1];
	send_data[2] = HEADER[2];
	send_data[3] = HEADER[3];
	send_data[4] = 0xFE;  // id
	send_data[5] = length & 0xFF;
	send_data[6] = (length>>8) & 0xFF;
	send_data[7] = INSTRUCTION_FAST_SYNC_READ;  // instruction
	send_data[8] = dp.address() & 0xFF;
	send_data[9] = (dp.address()>>8) & 0xFF;
	send_data[10] = dp.size() & 0xFF;
	send_data[11] = (dp.size()>>8) & 0xFF;
	for(int i_servo=0; i_servo<num_servo; i_servo++) {
		send_data[12+i_servo] = servo_id_list[i_servo];
	}
	uint16_t sum = CalcChecksum(send_data, 12+num_servo);
	send_data[12+num_servo] = sum & 0xFF;
	send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read fast Warn: clear buffer time out\n");
            break;
        }
    }
	port_handler_->clearPort();
	port_handler_->writePort(send_data, 14+num_servo);

    // データ読み込みの処理
	uint8_t length_a_servo = 4+dp.size(); // id error {data} crc crc
	uint16_t length_read_data = 8+length_a_servo*num_servo;
    timeout_last_read_ = false;
    port_handler_->setPacketTimeout( uint16_t(length_read_data) );
    while(port_handler_->getBytesAvailable() < length_read_data) {
		if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Fast Sync Read Error(time out) : available bytes %d / %d\n", port_handler_->getBytesAvailable(), length_read_data);
            deficient_byte_ = length_read_data - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return map<uint8_t, int64_t>();
		}
	}

	// パケットの読み込み
    uint8_t read_data[408]; // 読み込むサーボとデータサイズによって変わるが， (4+4)*50+8=408 100サーボに同時に読み込むことはないので十分
    int8_t read_length = port_handler_->readPort(read_data, length_read_data);
    // 全体のエラーチェック
    comm_error_last_read_ = false;
    if (read_length == -1) {
		if(verbose_) printf("Fast Sync Read Error(read port)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, int64_t>();
	}
	if (read_data[0] != HEADER[0] || // 読み込めたパケットのヘッダーの確認
	    read_data[1] != HEADER[1] ||
	    read_data[2] != HEADER[2] ||
	    read_data[3] != HEADER[3]) {
		if(verbose_) printf("Fast Sync Read Error(header)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, int64_t>();
	}
	if ( read_data[4] != 0xFE ) { // 読み込めたパケットのIDがブロードキャスト用のものか確認
		if(verbose_) printf("Fast Sync Read Error(broad cast id)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, int64_t>();
	}
	// 読み込めたパケットを個々のサーボのパケットに分割してエラーと正常系の処理,チェックサムの処理は行わない
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    map<uint8_t, int64_t> id_data_int_map;
	for(int i_servo=0; i_servo<num_servo; i_servo++) {
		uint8_t id = (uint8_t)read_data[9 + i_servo*length_a_servo]; // servo id
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Fast Sync Read Error(packet id) : expected %d return %d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            return map<uint8_t, int64_t>(); // これ以降すべての読み込みを諦める．
        }
		uint8_t error = (uint8_t)read_data[8 + i_servo*length_a_servo]; // error
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Fast Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue; // 他のidのデータは生きている可能性があるので続行
        }
		// 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
        if ( error & 0x80 ) hardware_error_last_read_ = true; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if ( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
		for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[10 + i_servo*length_a_servo + i];
		id_data_int_map[id] = DecodeDataRead(dp.data_type());
	}
	return id_data_int_map;
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param vector<DynamixelAddress> dp_list_unsorted 対象のパラメータのインスタンスのリスト
 * @param uint8_t servo_id 対象のID
 * @param vector<int64_t> data_int_list 書き込むデータのリスト．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::Write(const vector<DynamixelAddress>& dp_list_sorted, uint8_t servo_id, const vector<int64_t>& data_int_list) {
    if (dp_list_sorted.size() > 10) {
        if(verbose_) printf("Write Error(too many param): param num=%d > 10\n", (int)dp_list_sorted.size());
        return false;
    }
    if ( dp_list_sorted.size() != data_int_list.size() ) {
        if(verbose_) printf("Write Error(mismatch param and data num): param num=%d, data num=%d\n", (int)dp_list_sorted.size(), (int)data_int_list.size());
        return false;
    }
    // 書き込むデータの範囲を決定, ソート済みかつ連続していないとNG
    DynamixelAddress dp_min = *dp_list_sorted.begin();
    DynamixelAddress dp_max = *dp_list_sorted.rbegin();
    for (size_t i=0; i<dp_list_sorted.size()-1; i++) { // アドレスが連続しているか確認
        if (dp_list_sorted[i].address() + dp_list_sorted[i].size() != dp_list_sorted[i+1].address()) {
            if(verbose_) printf("Write Error(address is not continuous): ID %d\n", servo_id);
            return false;
        }
    }
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();
    uint8_t send_data[52] = {0}; // 書き込むdpの数とサイズによって変わるが， 4*10+12=52 4バイトのデータ10個を同時に書き込むことはないので十分
    uint16_t length = size_total_dp+5;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_WRITE;  // instruction
    send_data[8] = dp_min.address() & 0xFF;
    send_data[9] = (dp_min.address()>>8) & 0xFF;
    for (size_t i=0; i<dp_list_sorted.size(); i++) {
        const DynamixelAddress& dp = dp_list_sorted[i];
        uint8_t index = dp.address() - dp_min.address();
        EncodeDataWrite(dp.data_type(), data_int_list[i]);
        for(size_t i=0; i<dp.size(); i++) {
            send_data[10+index+i] = data_write_[i];
        }
    }
    uint16_t sum = CalcChecksum(send_data, 10+size_total_dp);
    send_data[10+size_total_dp] = sum & 0xFF;
    send_data[11+size_total_dp] = (sum>>8) & 0xFF;

    port_handler_->clearPort();
    port_handler_->writePort(send_data, 12+size_total_dp);

    if (status_return_level_ != 2) return true;
    // 以降はwriteと同じ

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout( uint16_t(11) );
    while(port_handler_->getBytesAvailable() < 11) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Write Error(return status time out): ID %d, available bytes %d/11\n", servo_id, port_handler_->getBytesAvailable());
            timeout_last_read_ = true;
            return false;
        }
    }
    uint8_t read_data[11]; //writeのステータスパケットは固定で11
    port_handler_->readPort(read_data, 11);
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
        if(verbose_) printf("Write Error(return status header or id): ID %d\n", servo_id);
        comm_error_last_read_ = true;
        return false;
    }
    uint8_t error = (uint8_t)read_data[8]; // error
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Write Error(return status packet error): ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return false;
    }
    uint16_t sum_est = CalcChecksum(read_data, 9);
    uint16_t sum_read = uint16_t(read_data[9]) | uint16_t(read_data[9+1])<<8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Write Error(return status crc): ID %d, est=%d, read=%d\n", servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return false;
    }
    // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    return true;
}


/** @fn
 * @brief Dynamixelから複数の情報を同時に読み込む
 * @param uint8_t servo_id 対象のID
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @return vector<int64_t> 読み込んだデータを格納する配列．intに変換済みのもの．
 */
vector<int64_t> DynamixelCommunicator::Read(const vector<DynamixelAddress>& dp_list, uint8_t servo_id) {
    if (dp_list.size() > 10) {
        if(verbose_) printf("Read Error(too many param): ID %d, param num=%d > 10\n", (int)servo_id, (int)dp_list.size());
        return vector<int64_t>();
    }
    // 読み込むデータの範囲を決定, 連続していなくても許容
    vector<DynamixelAddress> dp_list_sorted = dp_list;
    sort(dp_list_sorted.begin(), dp_list_sorted.end(), [](const DynamixelAddress& a, const DynamixelAddress& b) { return a.address() < b.address(); });
    DynamixelAddress dp_min = *dp_list_sorted.begin();
    DynamixelAddress dp_max = *dp_list_sorted.rbegin();
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();

    // instruction packetを作成
    uint8_t send_data[14] = {0}; // Readのインストラクションパケットは固定で14
    uint16_t length = 7;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_READ;  // instruction
    send_data[8] = dp_min.address() & 0xFF;
    send_data[9] = (dp_min.address()>>8) & 0xFF;
    send_data[10] = size_total_dp & 0xFF;
    send_data[11] = (size_total_dp>>8) & 0xFF;
    uint16_t sum = CalcChecksum(send_data, 12);
    send_data[12] = sum & 0xFF;
    send_data[13] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Read Warn: clear buffer time out\n");
            break;
        }
    }
    port_handler_->clearPort();
    port_handler_->writePort(send_data, 14);

    if (status_return_level_ == 0) return vector<int64_t>(dp_list.size(), 0); 

    // データ読み込みの処理
    timeout_last_read_ = false;
    port_handler_->setPacketTimeout( uint16_t(11+size_total_dp) );
    while(port_handler_->getBytesAvailable() < 11+size_total_dp) {
        if (port_handler_->isPacketTimeout()) {
        if(verbose_) printf("Read Error(time out): ID %d, available bytes %d / %d\n", servo_id, port_handler_->getBytesAvailable(), 11+size_total_dp);
        deficient_byte_ = 11+size_total_dp - port_handler_->getBytesAvailable();
        timeout_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0); // これ以降すべての読み込みを諦める．
        }
    }

    uint8_t read_data[51]; // 読み込むdpの数とサイズによって変わるが， 4*10+11=51 4バイトのデータ10個を同時に読み込むことはないので十分
    uint8_t read_length = port_handler_->readPort(read_data, 11+size_total_dp);
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_length != 11+size_total_dp) {
        if(verbose_) printf("Read Error(no data): ID %d, data_length = %d\n", servo_id, read_length);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3]) {
        if(verbose_) printf("Read Error(header): ID %d\n", servo_id);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    uint8_t id = read_data[4];
    if ( id != servo_id ) {
        if(verbose_) printf("Read Error(packet id) : ID %d\n", id);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    uint8_t error = (uint8_t)read_data[8];
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Read Error(packet error) : ID %d\n", id);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    uint16_t sum_est = CalcChecksum(read_data, 9+size_total_dp);
    uint16_t sum_read = uint16_t(read_data[9+size_total_dp]) | uint16_t(read_data[9+size_total_dp+1])<<8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Read Error(crc): ID %d, est=%d, read=%d\n", servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }

    // 正常なデータ
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    vector<int64_t> data_int_list(dp_list.size(), 0);
    for (size_t i_dp=0; i_dp<dp_list.size(); i_dp++) {
        const DynamixelAddress& dp = dp_list[i_dp];
        uint8_t index = dp.address() - dp_min.address();
        for(size_t i=0; i<dp.size(); i++) {
            data_read_[i] = read_data[9+index+i];
        }
        data_int_list[i_dp] = DecodeDataRead(dp.data_type());
    }
    return data_int_list;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, vector<int64_t>> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, vector<int64_t>> DynamixelCommunicator::SyncRead(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list) {
    if (servo_id_list.size() > 100) {
        if(verbose_) printf("Sync Read Error(too many servo): servo num=%d > 100\n", (int)servo_id_list.size());
        return map<uint8_t, vector<int64_t>>();
    }
    if (dp_list.size() > 10) {
        if(verbose_) printf("Sync Read Error(too many param): param num=%d > 10\n", (int)dp_list.size());
        return map<uint8_t, vector<int64_t>>();
    }
    // 読み込むデータの範囲を決定, 連続していなくても許容
    vector<DynamixelAddress> dp_list_sorted = dp_list;
    sort(dp_list_sorted.begin(), dp_list_sorted.end(), [](const DynamixelAddress& a, const DynamixelAddress& b) { return a.address() < b.address(); });
    DynamixelAddress dp_min = *dp_list_sorted.begin();
    DynamixelAddress dp_max = *dp_list_sorted.rbegin();
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();

    uint8_t num_servo = servo_id_list.size();
    uint8_t send_data[114] = {0}; // 読み込むサーボの数によって変わるが， 100+14=114 100サーボに同時に読み込むことはないので十分
    uint16_t length = num_servo+7;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = 0xFE;  // id
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_SYNC_READ;  // instruction
    send_data[8] = dp_min.address() & 0xFF;
    send_data[9] = (dp_min.address()>>8) & 0xFF;
    send_data[10] = size_total_dp & 0xFF;
    send_data[11] = (size_total_dp>>8) & 0xFF;
    for(int i_servo=0; i_servo<num_servo; i_servo++) {
        send_data[12+i_servo] = servo_id_list[i_servo];
    }
    uint16_t sum = CalcChecksum(send_data, 12+num_servo);
    send_data[12+num_servo] = sum & 0xFF;
    send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read Warn: clear buffer time out\n");
            break;
        }
    }
    port_handler_->clearPort();
    port_handler_->writePort(send_data, 14+num_servo);

    // if(verbose_) {printf("write:" ); for (size_t i=0; i<14+num_servo; i++) printf("%02X ", send_data[i]); printf("\n");}

    // データ読み込みの処理
    uint8_t read_data[51]; // 読み込むdpの数とサイズによって変わるが， 4*10+11=51 4バイトのデータ10個を同時に読み込むことはないので十分 
    map<uint8_t, vector<int64_t>> id_data_vec_map;
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    comm_error_last_read_ = false;
    for(int i_servo=0; i_servo<num_servo; i_servo++) {
        timeout_last_read_ = false;
        port_handler_->setPacketTimeout( uint16_t(11+size_total_dp) );
        while(port_handler_->getBytesAvailable() < 11+size_total_dp) {
            if (port_handler_->isPacketTimeout()) {
                if(verbose_) printf("Sync Read Error(time out): ID %d, available bytes %d / %d\n", servo_id_list[i_servo], port_handler_->getBytesAvailable(), 11+size_total_dp);
                deficient_byte_ = 11+size_total_dp - port_handler_->getBytesAvailable();
                timeout_last_read_ = true;
                return id_data_vec_map; // これ以降すべての読み込みを諦める．
            }
        }

        int8_t read_length = port_handler_->readPort(read_data, 11+size_total_dp);
        // if(verbose_) {printf("read:" ); for (size_t i=0; i<read_length; i++) printf("%02X ", read_data[i]); printf("\n");}

        // エラーチェック
        if (read_length == -1) {
        if(verbose_) printf("Sync Read Error(read port) : ID %d\n", servo_id_list[i_servo]);
        comm_error_last_read_ = true;
        continue;
        }
        if (read_data[0] != HEADER[0] or
            read_data[1] != HEADER[1] or
            read_data[2] != HEADER[2] or
            read_data[3] != HEADER[3]) {
        if(verbose_) printf("Sync Read Error(header): ID %d\n", servo_id_list[i_servo]);
        comm_error_last_read_ = true;
        continue;
        }
        uint8_t id = read_data[4];
        if ( id != servo_id_list[i_servo] ) {
        if(verbose_) printf("Sync Read Error(packet id) : est ID=%d, read ID=%d\n", servo_id_list[i_servo], id);
        comm_error_last_read_ = true;
        continue;
        }
        uint8_t error = (uint8_t)read_data[8]; 
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Sync Read Error(packet error) : ID %d\n", id);
        comm_error_last_read_ = true;
        continue;
        }
        uint16_t sum_est = CalcChecksum(read_data, 9+size_total_dp);
        uint16_t sum_read = uint16_t(read_data[9+size_total_dp]) | uint16_t(read_data[9+size_total_dp+1])<<8;
        if (sum_est != sum_read) {
        if(verbose_) printf("Sync Read Error(crc): ID %d, est=%d, read=%d\n", id, sum_est, sum_read);
        comm_error_last_read_ = true;
        continue;
        }
        
        // 正常なデータ
        if ( error & 0x80 ) hardware_error_last_read_ = true ; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if ( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
        id_data_vec_map[id].resize(dp_list.size());
        for (size_t i_dp=0; i_dp<dp_list.size(); i_dp++) {
            const DynamixelAddress& dp = dp_list[i_dp];
            uint8_t index = dp.address() - dp_min.address();
            for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[9+index+i];
            id_data_vec_map[id][i_dp] = DecodeDataRead(dp.data_type());
        }
    }
    return id_data_vec_map;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む,パケットを最小限にすることで高速化したもの．サーボ1つでも失敗するとすべて読み込めない
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, vector<int64_t>> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, vector<int64_t>> DynamixelCommunicator::SyncRead_fast(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list) {
    if ( servo_id_list.size() > 50 ) {
        if(verbose_) printf("Fast Sync Read Error(too many servo): servo num=%d > 50\n", (int)servo_id_list.size());
        return map<uint8_t, vector<int64_t>>();
    }
    if ( dp_list.size() > 10 ) {
        if(verbose_) printf("Fast Sync Read Error(too many param): param num=%d > 10\n", (int)dp_list.size());
        return map<uint8_t, vector<int64_t>>();
    }    
    // 読み込むデータの範囲を決定, 連続していなくても許容
    vector<DynamixelAddress> dp_list_sorted = dp_list;
    sort(dp_list_sorted.begin(), dp_list_sorted.end(), [](const DynamixelAddress& a, const DynamixelAddress& b) { return a.address() < b.address(); });
    DynamixelAddress dp_min = *dp_list_sorted.begin();
    DynamixelAddress dp_max = *dp_list_sorted.rbegin();
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();

	uint8_t send_data[64] = {0}; // 読み込むサーボの数によって変わるが， 50+14=64 50サーボに同時に読み込むことはないので十分 
	uint8_t num_servo = servo_id_list.size();
	uint16_t length = num_servo+7;
	send_data[0] = HEADER[0];
	send_data[1] = HEADER[1];
	send_data[2] = HEADER[2];
	send_data[3] = HEADER[3];
	send_data[4] = 0xFE;  // id
	send_data[5] = length & 0xFF;
	send_data[6] = (length>>8) & 0xFF;
	send_data[7] = INSTRUCTION_FAST_SYNC_READ;  // instruction
	send_data[8] = dp_min.address() & 0xFF;
	send_data[9] = (dp_min.address()>>8) & 0xFF;
	send_data[10] = size_total_dp & 0xFF;
	send_data[11] = (size_total_dp>>8) & 0xFF;
	for(int i_servo=0; i_servo<num_servo; i_servo++) {
		send_data[12+i_servo] = servo_id_list[i_servo];
	}
	uint16_t sum = CalcChecksum(send_data, 12+num_servo);
	send_data[12+num_servo] = sum & 0xFF;
	send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read fast Warn: clear buffer time out\n");
            break;
        }
    }
	port_handler_->clearPort();
	port_handler_->writePort(send_data, 14+num_servo);

    // データ読み込みの処理
 	uint8_t length_a_servo = 4+size_total_dp;
	uint16_t length_read_data = 8+length_a_servo*num_servo;
    timeout_last_read_ = false;
	port_handler_->setPacketTimeout( uint16_t(length_read_data) );
	while(port_handler_->getBytesAvailable() < length_read_data) {
		if (port_handler_->isPacketTimeout() ) {
		if(verbose_) printf("Fast Sync Read Error(time out) : available bytes %d / %d\n", port_handler_->getBytesAvailable(), length_read_data);
        deficient_byte_ = length_read_data - port_handler_->getBytesAvailable();
		timeout_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
		}
	}

	// パケットの読み込み
    uint8_t read_data[1458]; // 読み込むサーボとデータサイズによって変わるが， 最大でも同時読み込みしたいアドレスの幅は25程度なので，(25+4)*50+8=1458;
    int8_t read_length = port_handler_->readPort(read_data, length_read_data);

    // if(verbose_) {printf("write:" ); for (size_t i=0; i<14+num_servo; i++) printf("%02X ", send_data[i]); printf("\n");}
    // if(verbose_) {printf("read:" ); for (size_t i=0; i<read_length; i++) printf("%02X ", read_data[i]); printf("\n");}

    // 全体のエラーチェック
    comm_error_last_read_ = false;
	if (read_length == -1) {
		if(verbose_) printf("Fast Sync Read Error(read port)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
	}
	// 読み込めたパケットのヘッダーの確認
	if (read_data[0] != HEADER[0] ||
		read_data[1] != HEADER[1] ||
		read_data[2] != HEADER[2] ||
		read_data[3] != HEADER[3]) {
		if(verbose_) printf("Fast Sync Read Error(header)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
	}
	// 読み込めたパケットのIDがブロードキャスト用のものか確認
	if ( read_data[4] != 0xFE ) {
		if(verbose_) printf("Fast Sync Read Error(broad cast id)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
	}
	// 読み込めたパケットを個々のサーボのパケットに分割して処理,チェックサムの処理は行わない
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    map<uint8_t, vector<int64_t>> id_data_vec_map;
	for(int i_servo=0; i_servo<num_servo; i_servo++) {
		uint8_t id = (uint8_t)read_data[9 + i_servo*length_a_servo]; // servo id
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Fast Sync Read Error(packet id) : expected %d, return %d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            return map<uint8_t, vector<int64_t>>(); // これ以降すべての読み込みを諦める．
        }
		uint8_t error = (uint8_t)read_data[8 + i_servo*length_a_servo];
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Fast Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue; // 他のidのデータは生きている可能性があるので続行
        }
		// 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
        if( error & 0x80 ) hardware_error_last_read_ = true; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
        id_data_vec_map[id].resize(dp_list.size(), 0);
        for (size_t i_dp=0; i_dp<dp_list.size(); i_dp++) {
            const DynamixelAddress& dp = dp_list[i_dp];
            uint8_t index = dp.address() - dp_min.address();
            for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[10+i_servo*length_a_servo+index+i];
            id_data_vec_map[id][i_dp] = DecodeDataRead(dp.data_type());
        }
	}
	return id_data_vec_map;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報を書き込む
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param vector<uint8_t> servo_id_list 書き込むサーボのIDのリスト
 * @param vector<vector<int64_t>> data_vec_list 書き込むデータのリスト．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(const vector<DynamixelAddress>& dp_list_sorted, const vector<uint8_t>& servo_id_list, const vector<vector<int64_t>>& data_vec_list) {
  if (dp_list_sorted.size() > 10) {
    if(verbose_) printf("Sync Write Error(too many param): param num=%d > 10\n", (int)dp_list_sorted.size());
    return false;
  }
  if (servo_id_list.size() > 100) {
    if(verbose_) printf("Sync Write Error(too many servo): servo num=%d > 100\n", (int)servo_id_list.size());
    return false;
  }
  if (servo_id_list.size() != data_vec_list.size()) {
    if(verbose_) printf("Sync Write Error(mismatch servo and data_vec num): servo num=%d, data_vec num=%d\n", (int)servo_id_list.size(), (int)data_vec_list.size());
    return false;
  }
  for (auto data_vec : data_vec_list) if ( dp_list_sorted.size() != data_vec.size() ) {
    if(verbose_) printf("Sync Write Error(mismatch param and data num): param num=%d, data num=%d\n", (int)servo_id_list.size(), (int)data_vec.size());
    return false;
  }
  // 書き込むデータの範囲を決定, ソート済みかつ連続していないとNG
  DynamixelAddress dp_min = *dp_list_sorted.begin();
  DynamixelAddress dp_max = *dp_list_sorted.rbegin();
  // アドレスが連続しているか確認
  for (size_t i=0; i<dp_list_sorted.size()-1; i++) {
    if (dp_list_sorted[i].address() + dp_list_sorted[i].size() != dp_list_sorted[i+1].address()) {
    if(verbose_) printf("Sync Write Error(address is not continuous):");
    return false;
    }
  }
  auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();
  uint8_t send_data[1314] = {0}; // 書き込むdpの数とサイズとサーボの数によって変わるが， 最大でも同時読み込みしたいアドレスの幅は25程度なので，(25+1)*50+14=1314
  int8_t num_servo = servo_id_list.size();
  uint16_t length = (1+size_total_dp)*num_servo+7;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = 0xFE;  // id
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_SYNC_WRITE;  // instruction
  send_data[8] = dp_min.address() & 0xFF;
  send_data[9] = (dp_min.address()>>8) & 0xFF;
  send_data[10] = size_total_dp & 0xFF;
  send_data[11] = (size_total_dp>>8) & 0xFF;
  for(int i_servo=0; i_servo<num_servo; i_servo++) {
    send_data[12+i_servo*(size_total_dp+1)] = servo_id_list[i_servo];
    for (size_t i_dp=0; i_dp<dp_list_sorted.size(); i_dp++) {
      const DynamixelAddress& dp = dp_list_sorted[i_dp];
      uint8_t index = dp.address() - dp_min.address();
      EncodeDataWrite(dp.data_type(), data_vec_list[i_servo][i_dp]);
      for(int i_data=0; i_data<dp.size(); i_data++) {
        send_data[12+i_servo*(size_total_dp+1)+1+index+i_data] = data_write_[i_data];
      }
    }
  }

  uint16_t sum = CalcChecksum(send_data, 12+num_servo*(size_total_dp+1));
  send_data[12+num_servo*(size_total_dp+1)] = sum & 0xFF;
  send_data[13+num_servo*(size_total_dp+1)] = (sum>>8) & 0xFF;

  port_handler_->clearPort();
  port_handler_->writePort(send_data, 14+num_servo*(size_total_dp+1));
  return true;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param map<uint8_t, int64_t> id_data_int_map 書き込むサーボのIDと書き込むデータのマップ
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(const vector<DynamixelAddress>& dp_list, const map<uint8_t, vector<int64_t>>& id_data_int_map) {
    vector<uint8_t>         servo_id_list; servo_id_list.reserve(id_data_int_map.size());
    vector<vector<int64_t>> data_vec_list; data_vec_list.reserve(id_data_int_map.size());
    for (const auto& id_data : id_data_int_map) {
        servo_id_list.push_back(id_data.first);    // キー（サーボID）を追加
        data_vec_list.push_back(id_data.second);   // 値（データ）を追加
    }
    return SyncWrite( dp_list, servo_id_list, data_vec_list );
}
