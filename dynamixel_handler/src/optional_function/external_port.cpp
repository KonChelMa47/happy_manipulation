#include "external_port.hpp"

#include "myUtils/formatting_output.hpp"
#include "myUtils/make_iterator_convenient.hpp"

#define ROS_INFO(...)  RCLCPP_INFO(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(parent_.get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(parent_.get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(...)  RCLCPP_WARN_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_STOP(...) \
    {{ RCLCPP_FATAL(parent_.get_logger(), __VA_ARGS__); rclcpp::shutdown(); std::exit(EXIT_FAILURE); }}

void DynamixelHandler::ExternalPort::CallbackExternalPort(const DxlExternalPort::SharedPtr msg){
    static auto& id_set_ = parent_.id_set_;
    static auto& model_ = parent_.model_;

    // msg.id_list内のIDとmsg.port内のport番号の妥当性を確認
    vector<size_t> valid_indice;
    vector<uint8_t> valid_id_list, valid_port_list;
    if ( msg->id_list.size() != msg->port.size() ) {
         ROS_WARN("External Port, ID and port size mismatch"); return;
    } // サイズ違いは問答無用でNG
    for (size_t i=0; i<msg->id_list.size(); i++) {
        auto id = msg->id_list[i];
        if ( !is_in(id, id_set_) ) continue; // 存在しないIDは無視
        if ( !has_external_port( model_[id]) ){
            ROS_WARN("ID [%d] has no external port", id); continue;
        }
        auto port = msg->port[i];
        if ( !is_in( port, ex_port_indice_[series_[id]]) ){
            ROS_WARN("ID [%d] does not have external port_%d", id, port); continue;
        }
        // 重複を取り除く
        if ( is_in( id, valid_id_list) && is_in( port, valid_port_list) ) continue;
        valid_indice.push_back(i);
        valid_id_list.push_back(id);
        valid_port_list.push_back(port);
    }
    if ( valid_indice.empty() ) return; // 有効なIDがない場合は何もしない
    // mode, data それぞれの指令が入力されたかどうかを確認
    const bool has_data = msg->id_list.size() == msg->data.size();
    const bool has_mode = msg->id_list.size() == msg->mode.size();
    if (verbose_callback_) { 
        ROS_INFO("External Port, '%zu' port(s) are tryed to updated", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list,   "  ID  ")); 
        ROS_INFO_STREAM(id_list_layout(valid_port_list, "  port")); 
        if ( has_data ) ROS_INFO("  - updated: data"); else if ( !msg->data.empty() ) ROS_WARN("  - skipped: data (size mismatch)");
        if ( has_mode ) ROS_INFO("  - updated: mode"); else if ( !msg->mode.empty() ) ROS_WARN("  - skipped: mode (size mismatch)");
    }
    // データの反映
    for (auto i : valid_indice) {
        auto   ID = msg->id_list[i];
        auto port = msg->port[i];
        touched_id_export_.insert(ID);
        if ( has_data ) {
            export_w_[ID][port].data = msg->data[i];
            updated_id_export_.data.insert(ID);
        }
        if ( has_mode ) {
                 if ( msg->mode[i] == msg->MODE_ANALOG_IN           ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_AIN;
            else if ( msg->mode[i] == msg->MODE_DIGITAL_OUT         ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_DOUT;
            else if ( msg->mode[i] == msg->MODE_DIGITAL_IN_PULLUP   ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_DIN_PULLUP;
            else if ( msg->mode[i] == msg->MODE_DIGITAL_IN_PULLDOWN ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_DIN_PULLDOWN;
            else {continue; ROS_WARN("  Invalid mode [%s]", msg->mode[i].c_str());} // verbose_callback_ に関わらず出力
            updated_id_export_.mode.insert(ID);
        }
    }
    if ( verbose_callback_ ) ROS_INFO("------------------+------------------");
}


void DynamixelHandler::ExternalPort::BroadcastExternalPort(){
    static auto& id_set_ = parent_.id_set_;
    DxlExternalPort msg;
    for ( const auto& [id, export_map] : export_r_ ) if ( is_in(id, id_set_) ) {
        for (const auto& [port, contents] : export_map) {
            msg.id_list.push_back(id);
            msg.port.push_back(port);
            msg.data.push_back(contents.data);
            switch ( contents.mode ){
                case EXTERNAL_PORT_MODE_AIN         : msg.mode.push_back(msg.MODE_ANALOG_IN          ); break;
                case EXTERNAL_PORT_MODE_DOUT        : msg.mode.push_back(msg.MODE_DIGITAL_OUT        ); break;
                case EXTERNAL_PORT_MODE_DIN_PULLUP  : msg.mode.push_back(msg.MODE_DIGITAL_IN_PULLUP  ); break;
                case EXTERNAL_PORT_MODE_DIN_PULLDOWN: msg.mode.push_back(msg.MODE_DIGITAL_IN_PULLDOWN); break;
            }
        }
    }

    if(pub_ex_port_) pub_ex_port_->publish(msg);
}

uint16_t DynamixelHandler::ExternalPort::ReadExternalPortMode(uint8_t id, uint8_t port){
    DynamixelAddress addr = AddrX::external_port_data_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_data_1; break;
        case 2: addr = AddrX::external_port_data_2; break;
        case 3: addr = AddrX::external_port_data_3; break;
        default: return 0;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_data_1; break;
        case 2: addr = AddrP::external_port_data_2; break;
        case 3: addr = AddrP::external_port_data_3; break;
        case 4: addr = AddrP::external_port_data_4; break;
        default: return 0;
    } else       return 0;
    return parent_.dyn_comm_.tryRead(addr, id);
}

uint16_t DynamixelHandler::ExternalPort::ReadExternalPortData(uint8_t id, uint8_t port){
    DynamixelAddress addr = AddrX::external_port_mode_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_mode_1; break;
        case 2: addr = AddrX::external_port_mode_2; break;
        case 3: addr = AddrX::external_port_mode_3; break;
        default: return 0;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_mode_1; break;
        case 2: addr = AddrP::external_port_mode_2; break;
        case 3: addr = AddrP::external_port_mode_3; break;
        case 4: addr = AddrP::external_port_mode_4; break;
        default: return 0;
    } else       return 0;
    return parent_.dyn_comm_.tryRead(addr, id);
}

bool DynamixelHandler::ExternalPort::WriteExternalPortMode(uint8_t id, uint8_t port, uint16_t data){
    DynamixelAddress addr = AddrX::external_port_data_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_data_1; break;
        case 2: addr = AddrX::external_port_data_2; break;
        case 3: addr = AddrX::external_port_data_3; break;
        default: return false;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_data_1; break;
        case 2: addr = AddrP::external_port_data_2; break;
        case 3: addr = AddrP::external_port_data_3; break;
        case 4: addr = AddrP::external_port_data_4; break;
        default: return false;
    } else       return false;
    return parent_.dyn_comm_.tryWrite(addr, id, data);
}

bool DynamixelHandler::ExternalPort::WriteExternalPortData(uint8_t id, uint8_t port, uint16_t data){
    DynamixelAddress addr = AddrX::external_port_mode_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_mode_1; break;
        case 2: addr = AddrX::external_port_mode_2; break;
        case 3: addr = AddrX::external_port_mode_3; break;
        default: return false;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_mode_1; break;
        case 2: addr = AddrP::external_port_mode_2; break;
        case 3: addr = AddrP::external_port_mode_3; break;
        case 4: addr = AddrP::external_port_mode_4; break;
        default: return false;
    } else       return false;
    return parent_.dyn_comm_.tryWrite(addr, id, data);
}

template <> void DynamixelHandler::ExternalPort::SyncWriteExternalPortMode(unordered_set<uint8_t> updated_id_mode){
    static auto& dyn_comm_ = parent_.dyn_comm_;
    map<uint8_t, int64_t> id_torque_map_x, id_torque_map_p;
    for (auto id : updated_id_mode) {
        if ( series_[id]==AddrX::series() ) id_torque_map_x[id] = TORQUE_DISABLE;
        if ( series_[id]==AddrP::series() ) id_torque_map_p[id] = TORQUE_DISABLE;
    }
    if ( !id_torque_map_x.empty() ) dyn_comm_.SyncWrite(AddrX::torque_enable, id_torque_map_x);
    if ( !id_torque_map_p.empty() ) dyn_comm_.SyncWrite(AddrP::torque_enable, id_torque_map_p);
    SyncWriteExternalPortMode<AddrX>(updated_id_mode);
    SyncWriteExternalPortMode<AddrP>(updated_id_mode);
}
template <typename Addr> void DynamixelHandler::ExternalPort::SyncWriteExternalPortMode(unordered_set<uint8_t> updated_id_mode){
    static auto& width_log_ = parent_.width_log_;
    static auto& model_ = parent_.model_;
    static auto& dyn_comm_ = parent_.dyn_comm_;
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> exmode_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_exmode_vec_map; // id と 書き込むデータのベクタのマップ
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto port : port_list) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch ( port ) {
            case 1 : exmode_addr_list.push_back(Addr::external_port_mode_1 ); break;
            case 2 : exmode_addr_list.push_back(Addr::external_port_mode_2 ); break;
            case 3 : exmode_addr_list.push_back(Addr::external_port_mode_3 ); break;
            case 4 : if (Addr::series()==SERIES_P) {
                        exmode_addr_list.push_back(AddrP::external_port_mode_4 ); break;
                    } [[fallthrough]];
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
        }
        const auto& addr = exmode_addr_list.back();
        for (auto id : updated_id_mode) if ( series_[id]==Addr::series() ) //updated_id_modeがex_portを持つ妥当なサーボのIDを持つことは，CallbackExternalPortで担保されている
            id_exmode_vec_map[id].push_back( addr.val2pulse( export_w_[id][port].mode, model_[id] ) ); //export_w_がすべてのID全てのportに対して妥当な値を持っていることは初期化時に担保する
    }
    if ( id_exmode_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //* id_exmode_vec_mapの中身を確認
    if ( verbose_write_ ) {
        char header[100]; sprintf(header, "'%zu' servo(s) will be written", id_exmode_vec_map.size());
        auto ss = control_table_layout(width_log_, id_exmode_vec_map, exmode_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //*SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(exmode_addr_list, id_exmode_vec_map);
}

template <> void DynamixelHandler::ExternalPort::SyncWriteExternalPortData(unordered_set<uint8_t> updated_id_data){
    SyncWriteExternalPortData<AddrX>(updated_id_data);
    SyncWriteExternalPortData<AddrP>(updated_id_data);
}
template <typename Addr> void DynamixelHandler::ExternalPort::SyncWriteExternalPortData(unordered_set<uint8_t> updated_id_data){
    static auto& width_log_ = parent_.width_log_;
    static auto& model_ = parent_.model_;
    static auto& dyn_comm_ = parent_.dyn_comm_;
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> exdata_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_exdata_vec_map; // id と 書き込むデータのベクタのマップ
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto port : port_list) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch ( port ) {
            case 1 : exdata_addr_list.push_back(Addr::external_port_data_1 ); break;
            case 2 : exdata_addr_list.push_back(Addr::external_port_data_2 ); break;
            case 3 : exdata_addr_list.push_back(Addr::external_port_data_3 ); break;
            case 4 : if (Addr::series()==SERIES_P) { // ここでPシリーズであることの確認をするのは念のため, port_listの段階でport=4にはPしか来ないはず
                        exdata_addr_list.push_back(AddrP::external_port_data_4 ); break;
                    } [[fallthrough]];
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
        }
        const auto& addr = exdata_addr_list.back();
        for (auto id : updated_id_data) if ( series_[id]==Addr::series() ) //updated_id_dataがex_portを持つ妥当なサーボのIDを持つことは，CallbackExternalPortで担保されている
            id_exdata_vec_map[id].push_back( addr.val2pulse( export_w_[id][port].mode, model_[id] ) ); //export_w_がすべてのID全てのportに対して妥当な値を持っていることは初期化時に担保する
    }
    if ( id_exdata_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //* id_exdata_vec_mapの中身を確認
    if ( verbose_write_ ) {
        char header[100]; sprintf(header, "'%zu' servo(s) will be written", id_exdata_vec_map.size());
        auto ss = control_table_layout(width_log_, id_exdata_vec_map, exdata_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //*SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(exdata_addr_list, id_exdata_vec_map);
}


template <> double DynamixelHandler::ExternalPort::SyncReadExternalPortMode(set<uint8_t> id_set){
    map<DynamixelSeries, size_t> num;
    for (auto id : id_set) num[series_[id]]++;
    double suc_rate_X = SyncReadExternalPortMode<AddrX>(id_set);
    double suc_rate_P = SyncReadExternalPortMode<AddrP>(id_set);
    return (suc_rate_X * num[SERIES_X] + suc_rate_P * num[SERIES_P]) / (num[SERIES_X]+num[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::ExternalPort::SyncReadExternalPortMode(set<uint8_t> id_set){
    static auto& dyn_comm_ = parent_.dyn_comm_;
    static auto& width_log_ = parent_.width_log_;
    static auto& model_ = parent_.model_;
    static auto& use_fast_read_ = parent_.use_fast_read_;
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> exmode_addr_list;
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto p : port_list) switch ( p ) {
        case 1 : exmode_addr_list.push_back(Addr::external_port_mode_1 ); break;
        case 2 : exmode_addr_list.push_back(Addr::external_port_mode_2 ); break;
        case 3 : exmode_addr_list.push_back(Addr::external_port_mode_3 ); break;
        case 4 : if (Addr::series()==SERIES_P) { // ここでPシリーズであることの確認をするのは念のため, port_listの段階でport=4にはPしか来ないはず
                    exmode_addr_list.push_back(AddrP::external_port_mode_4 ); break;
                } [[fallthrough]];
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_exmode_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(exmode_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (exmode_addr_list, target_id_list);  fflush(stdout);
    const int N_total = target_id_list.size();
    const int N_suc   = id_exmode_vec_map.size();
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( verbose_read_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_exmode_vec_map.find(id) == id_exmode_vec_map.end() ) failed_id_list.push_back(id);
        ROS_WARN("'%d' servo(s) failed to read %s", N_total - N_suc, is_timeout ? " (time out)" : " (some kind packet error)");
        ROS_WARN_STREAM(id_list_layout(failed_id_list));
    }
    if ( verbose_read_ ) if ( N_suc>0 ) {
        char header[100]; sprintf(header, "'%d' servo(s) are read", N_suc);
        auto ss = control_table_layout(width_log_, id_exmode_vec_map, exmode_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // export_r_に反映
    for ( size_t i = 0; i < port_list.size(); i++) {
        DynamixelAddress addr = exmode_addr_list[i];
        for (const auto& [id, data_int] : id_exmode_vec_map)
            export_r_[id][port_list[i]].mode = addr.pulse2val( data_int[i], model_[id] );
    }
    return N_suc/(double)N_total;
}

template <> double DynamixelHandler::ExternalPort::SyncReadExternalPortData(set<uint8_t> id_set){
    map<DynamixelSeries, size_t> num;
    for (auto id : id_set) num[series_[id]]++;
    double suc_rate_X = SyncReadExternalPortData<AddrX>(id_set);
    double suc_rate_P = SyncReadExternalPortData<AddrP>(id_set);
    return (suc_rate_X * num[SERIES_X] + suc_rate_P * num[SERIES_P]) / (num[SERIES_X]+num[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::ExternalPort::SyncReadExternalPortData(set<uint8_t> id_set){
    static auto& dyn_comm_ = parent_.dyn_comm_;
    static auto& width_log_ = parent_.width_log_;
    static auto& model_ = parent_.model_;
    static auto& use_fast_read_ = parent_.use_fast_read_;
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> exdata_addr_list;
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto p : port_list) switch ( p ) {
        case 1 : exdata_addr_list.push_back(Addr::external_port_data_1 ); break;
        case 2 : exdata_addr_list.push_back(Addr::external_port_data_2 ); break;
        case 3 : exdata_addr_list.push_back(Addr::external_port_data_3 ); break;
        case 4 : if (Addr::series()==SERIES_P) { // ここでPシリーズであることの確認をするのは念のため, port_listの段階でport=4にはPしか来ないはず
                    exdata_addr_list.push_back(AddrP::external_port_data_4 ); break;
                } [[fallthrough]];
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_exdata_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(exdata_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (exdata_addr_list, target_id_list);  fflush(stdout);
    const int N_total = target_id_list.size();
    const int N_suc   = id_exdata_vec_map.size();
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( verbose_read_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_exdata_vec_map.find(id) == id_exdata_vec_map.end() ) failed_id_list.push_back(id);
        ROS_WARN("'%d' servo(s) failed to read %s", N_total - N_suc, is_timeout ? " (time out)" : " (some kind packet error)");
        ROS_WARN_STREAM(id_list_layout(failed_id_list));
    }
    if ( verbose_read_ ) if ( N_suc>0 ) {
        char header[100]; sprintf(header, "'%d' servo(s) are read", N_suc);
        auto ss = control_table_layout(width_log_, id_exdata_vec_map, exdata_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // export_r_に反映
    for ( size_t i = 0; i < port_list.size(); i++) {
        DynamixelAddress addr = exdata_addr_list[i];
        for (const auto& [id, data_int] : id_exdata_vec_map)
            export_r_[id][port_list[i]].data = addr.pulse2val( data_int[i], model_[id] );
    }
    return N_suc/(double)N_total;
}


DynamixelHandler::ExternalPort::ExternalPort(DynamixelHandler& parent) : parent_(parent) { // 本体を参照で保持, constを付けるとdyn_comm_
    ROS_INFO(" < Initializing External Port function ...   >");

    parent_.get_parameter_or("option/external_port.pub_ratio/mode", pub_ratio_mode_, 100u);
    parent_.get_parameter_or("option/external_port.pub_ratio/data", pub_ratio_data_, 10u);
    parent_.get_parameter_or("option/external_port/verbose/callback", verbose_callback_, false);
    parent_.get_parameter_or("option/external_port/verbose/write"   , verbose_write_   , false);
    parent_.get_parameter_or("option/external_port/verbose/read/raw", verbose_read_    , false);
    parent_.get_parameter_or("option/external_port/verbose/read/err", verbose_read_err_, false);

    pub_ex_port_ = parent_.create_publisher<DxlExternalPort>("dynamixel/external_port/read", 4);
    sub_ex_port_ = parent_.create_subscription<DxlExternalPort>("dynamixel/external_port/write", 4, bind(&DynamixelHandler::ExternalPort::CallbackExternalPort, this, _1));

    ROS_INFO(" < ... External Port function is initialized >");
}
DynamixelHandler::ExternalPort::~ExternalPort(){ // デストラクタ,  終了処理を行う

}
void DynamixelHandler::ExternalPort::MainProccess(){
    static auto& ping_err_ = parent_.ping_err_;
    static int cnt = -1; cnt++;

    SyncWriteExternalPortMode(updated_id_export_.mode);
    updated_id_export_.mode.clear();
    SyncWriteExternalPortData(updated_id_export_.data);
    updated_id_export_.data.clear();

    double success_rate = 0;
    set<uint8_t> target_id_set; for (auto id : touched_id_export_) if ( ping_err_[id]==0 ) target_id_set.insert(id);
    if ( pub_ratio_mode_ && cnt % pub_ratio_mode_ == 0 )
        success_rate += SyncReadExternalPortMode(target_id_set);
    if ( pub_ratio_data_ && cnt % pub_ratio_data_ == 0 )
        success_rate += SyncReadExternalPortData(target_id_set);
    if ( success_rate > 0.0 )
        BroadcastExternalPort();
}