#ifndef XmlManagerWindow_H
#define XmlManagerWindow_H

#include <QMainWindow>
#include <QListWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QMessageBox>
#include "ui_XmlConfigManager.h"
#include "ConfigManager.h"

/**
 * @brief 主窗口类
 * 负责显示配置参数界面
 */
class XmlManagerWindow : public QWidget
{
    Q_OBJECT
    
public:
    /**
     * @brief 构造函数
     * @param parent 父对象
     */
    XmlManagerWindow(QWidget* parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~XmlManagerWindow();
    
private:
    /**
     * @brief 初始化用户界面
     */
    void initUI();
    void BindAction();

private slots:
    /**
     * @brief 处理参数列表项变更
     * @param current 当前选中项
     * @param previous 之前选中项
     */
    void onParameterListCurrentItemChanged(QListWidgetItem* current, QListWidgetItem* previous);
    void onSaveButtonClicked();
    void onCancelButtonClicked();
    void onApplyButtonClicked();



private:
    Ui::XmlConfigManager ui;
};

#endif // XmlManagerWindow_H