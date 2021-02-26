#pragma once

#include <QtCore/QThread>
#include <rclcpp/rclcpp.hpp>

class NodeThread : public QThread {
    Q_OBJECT

public:
    /**
     * コンストラクタ
     * @param parent QThreadの親オブジェクト
     * @param node NodeThreadクラスが別スレッドで処理を行うrclcpp::Node
     */
    NodeThread(QObject *parent, std::shared_ptr<rclcpp::Node> node) : QThread(parent), _Node(node) {}

    /**
     * デストラクタ
     */
    virtual ~NodeThread(){}

    /**
     * rclcpp::Nodeを取得する
     * @return rclcpp::Node
     */
    std::shared_ptr<rclcpp::Node> node(void) const {
        return _Node;
    }

    /// quit()を実行してから終了するまでの待ち時間
    static constexpr int QUIT_TIMEOUT = 100;

private:
    void run(void) override;

    std::shared_ptr<rclcpp::Node> _Node;
};
