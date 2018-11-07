/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "treemodel.h"
//#include <iostream>
//#include <fstream>
#include <QTextStream>

#include <QFile>
#include <QDateTime>
#include <vector>

struct pub_sub_info
{
    QString name;
    QString topic;
    QString type;
    QString default_value;
    QString description;
    QString buflen;
};

//Declare?ing Functions
void headding(QTextStream &out, QString FILE_NAME);
void comment_subs(QTextStream &out, std::vector<pub_sub_info> subs);
void comment_pubs(QTextStream &out, std::vector<pub_sub_info> pubs);
void comment_params(QTextStream &out, std::vector<pub_sub_info> params);
void ROS_node_and_pub(QTextStream &out, std::vector<pub_sub_info> pubs);
void ROS_topics(QTextStream &out, std::vector<pub_sub_info> subs);
void ROS_callbacks(QTextStream &out, std::vector<pub_sub_info> subs);
void Params(QTextStream &out, std::vector<pub_sub_info> params);
void main_init_ROS(QTextStream &out, QString FILE_NAME);
void main_subs(QTextStream &out, std::vector<pub_sub_info> subs);
void main_pubs(QTextStream &out,std::vector<pub_sub_info> pubs);
void main_params_spin_end(QTextStream &out,std::vector<pub_sub_info> params);
void many_sub_callbacks(QTextStream &out, std::vector<pub_sub_info> subs);


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUi(this);

    QStringList headers;
    headers << tr("Item") << tr("Name") << tr("Type") << tr("Topic") << tr("Default Value") << tr("Description") << tr("Buffer Length");

    QFile file(":/default.txt");
    file.open(QIODevice::ReadOnly);
    TreeModel *model = new TreeModel(headers, file.readAll());
    file.close();

    view->setModel(model);
    for (int column = 0; column < model->columnCount(); ++column)
        view->resizeColumnToContents(column);

    connect(exitAction, &QAction::triggered, qApp, &QCoreApplication::quit);

    connect(view->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::updateActions);

    connect(actionsMenu, &QMenu::aboutToShow, this, &MainWindow::updateActions);
    connect(insertRowAction, &QAction::triggered, this, &MainWindow::insertRow);
    connect(insertColumnAction, &QAction::triggered, this, &MainWindow::insertColumn);
    connect(removeRowAction, &QAction::triggered, this, &MainWindow::removeRow);
    connect(removeColumnAction, &QAction::triggered, this, &MainWindow::removeColumn);
    connect(insertChildAction, &QAction::triggered, this, &MainWindow::insertChild);

    updateActions();
}

void MainWindow::insertChild()
{
    QModelIndex index = view->selectionModel()->currentIndex();
    QAbstractItemModel *model = view->model();

    if (model->columnCount(index) == 0) {
        if (!model->insertColumn(0, index))
            return;
    }

    if (!model->insertRow(0, index))
        return;

    for (int column = 0; column < model->columnCount(index); ++column) {
        QModelIndex child = model->index(0, column, index);
        model->setData(child, QVariant("[No data]"), Qt::EditRole);
        if (!model->headerData(column, Qt::Horizontal).isValid())
            model->setHeaderData(column, Qt::Horizontal, QVariant("[No header]"), Qt::EditRole);
    }

    view->selectionModel()->setCurrentIndex(model->index(0, 0, index),
                                            QItemSelectionModel::ClearAndSelect);
    updateActions();
}

bool MainWindow::insertColumn()
{
    QAbstractItemModel *model = view->model();
    int column = view->selectionModel()->currentIndex().column();

    // Insert a column in the parent item.
    bool changed = model->insertColumn(column + 1);
    if (changed)
        model->setHeaderData(column + 1, Qt::Horizontal, QVariant("[No header]"), Qt::EditRole);

    updateActions();

    return changed;
}

void MainWindow::insertRow()
{
    QModelIndex index = view->selectionModel()->currentIndex();
    QAbstractItemModel *model = view->model();

    if (!model->insertRow(index.row()+1, index.parent()))
        return;

    updateActions();

    for (int column = 0; column < model->columnCount(index.parent()); ++column) {
        QModelIndex child = model->index(index.row()+1, column, index.parent());
        model->setData(child, QVariant("[No data]"), Qt::EditRole);
    }
}

bool MainWindow::removeColumn()
{
    QAbstractItemModel *model = view->model();
    int column = view->selectionModel()->currentIndex().column();

    // Insert columns in each child of the parent item.
    bool changed = model->removeColumn(column);

    if (changed)
        updateActions();

    return changed;
}

void MainWindow::removeRow()
{
    QModelIndex index = view->selectionModel()->currentIndex();
    QAbstractItemModel *model = view->model();
    if (model->removeRow(index.row(), index.parent()))
        updateActions();
}

void MainWindow::updateActions()
{
    bool hasSelection = !view->selectionModel()->selection().isEmpty();
    removeRowAction->setEnabled(hasSelection);
    removeColumnAction->setEnabled(hasSelection);

    bool hasCurrent = view->selectionModel()->currentIndex().isValid();
    insertRowAction->setEnabled(hasCurrent);
    insertColumnAction->setEnabled(hasCurrent);

    if (hasCurrent) {
        view->closePersistentEditor(view->selectionModel()->currentIndex());

        int row = view->selectionModel()->currentIndex().row();
        int column = view->selectionModel()->currentIndex().column();
        if (view->selectionModel()->currentIndex().parent().isValid())
            statusBar()->showMessage(tr("Position: (%1,%2)").arg(row).arg(column));
        else
            statusBar()->showMessage(tr("Position: (%1,%2) in top level").arg(row).arg(column));
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    //This function is what happens after "generate" button is pressed.
    QAbstractItemModel *model = view->model();



    std::vector<pub_sub_info> pubs;
    std::vector<pub_sub_info> subs;
    std::vector<pub_sub_info> params;

    int param_type = 1;  //0 for pub //1 for sub
    int column = 0;
    int num_child = model->rowCount(model->index(param_type,column));

    for (int child = 0; child < num_child; child++)
    {
        QModelIndex child_name = model->index(param_type,column).child(child,1);
        QModelIndex child_type = model->index(param_type,column).child(child,2);
        QModelIndex child_topic = model->index(param_type,column).child(child,3);
        QModelIndex child_default_value = model->index(param_type,column).child(child,4);
        QModelIndex child_descripion = model->index(param_type,column).child(child,5);
        QModelIndex child_buflen = model->index(param_type,column).child(child,6);

        pub_sub_info sub;
        sub.name = model->data(child_name).toString();
        sub.type = model->data(child_type).toString();
        sub.topic = model->data(child_topic).toString();
        sub.default_value = model->data(child_default_value).toString();
        sub.description = model->data(child_descripion).toString();
        sub.buflen = model->data(child_buflen).toString();
        subs.push_back(sub);
    }
    param_type = 0; //now for publishers
    num_child = model->rowCount(model->index(param_type,column));

    for (int child = 0; child < num_child; child++)
    {
        QModelIndex child_name = model->index(param_type,column).child(child,1);
        QModelIndex child_type = model->index(param_type,column).child(child,2);
        QModelIndex child_topic = model->index(param_type,column).child(child,3);
        QModelIndex child_default_value = model->index(param_type,column).child(child,4);
        QModelIndex child_descripion = model->index(param_type,column).child(child,5);
        QModelIndex child_buflen = model->index(param_type,column).child(child,6);

        pub_sub_info pub;
        pub.name = model->data(child_name).toString();
        pub.type = model->data(child_type).toString();
        pub.topic = model->data(child_topic).toString();
        pub.default_value = model->data(child_default_value).toString();
        pub.description = model->data(child_descripion).toString();
        pub.buflen = model->data(child_buflen).toString();
        pubs.push_back(pub);
    }

    param_type = 2; //params //TODO:change format/names later?
    num_child = model->rowCount(model->index(param_type,column));

    for (int child = 0; child < num_child; child++)
    {
        QModelIndex child_name = model->index(param_type,column).child(child,1);
        QModelIndex child_type = model->index(param_type,column).child(child,2);
        QModelIndex child_topic = model->index(param_type,column).child(child,3);
        QModelIndex child_default_value = model->index(param_type,column).child(child,4);
        QModelIndex child_descripion = model->index(param_type,column).child(child,5);
        QModelIndex child_buflen = model->index(param_type,column).child(child,6);

        pub_sub_info param;
        param.name = model->data(child_name).toString();
        param.type = model->data(child_type).toString();
        param.topic = model->data(child_topic).toString();
        param.default_value = model->data(child_default_value).toString();
        param.description = model->data(child_descripion).toString();
        param.buflen = model->data(child_buflen).toString();
        params.push_back(param);
    }


    //opening file
    QString node_file_name = lineEdit_2->text();
    std::string file_name = node_file_name.toStdString();

    //saving a backup
    QFile::copy(node_file_name, "backup.txt");

    QFile file(node_file_name);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    QTextStream out(&file);

    //Now, call all the functions that build the file
    headding(out, node_file_name);
    comment_subs(out, subs);
    comment_pubs(out, pubs);
    comment_params(out, params);
    ROS_node_and_pub(out, pubs);
    ROS_topics(out, subs);
    ROS_callbacks(out, subs);
    Params(out, params);
    main_init_ROS(out, node_file_name);
    main_subs(out, subs);
    main_pubs(out, pubs);
    main_params_spin_end(out, params);
    many_sub_callbacks(out, subs);
    //comment_subs(out,subs);

    /*
    for (int child = 0; child < num_child; child++)
    {
        QModelIndex child_name = model->index(param_type,column).child(child,1);
        QModelIndex child_type = model->index(param_type,column).child(child,2);
        QModelIndex child_topic = model->index(param_type,column).child(child,3);
        QModelIndex child_default_value = model->index(param_type,column).child(child,4);
        QModelIndex child_descripion = model->index(param_type,column).child(child,5);


        QString d_child_name = model->data(child_name).toString();
        QString d_child_type = model->data(child_type).toString();
        QString d_child_topic = model->data(child_topic).toString();
        QString d_child_default_value = model->data(child_default_value).toString();
        QString d_child_descripion = model->data(child_descripion).toString();
        out << "    " << d_child_name << " ("<<d_child_type<<"): "<<d_child_topic << "\n"<<"        "<<d_child_descripion<<"\n";
    }
    //Now, Publishers
    */

    /*
    for (int param_type = 0; param_type < 3; param_type++)
    {
        //int column = 0;
        //int num_child = model->rowCount(model->index(param_type,column));

        for (int child = 0; child < num_child; child++)
        {
            QModelIndex child_name = model->index(param_type,column).child(child,1);
            QModelIndex child_type = model->index(param_type,column).child(child,2);
            QModelIndex child_topic = model->index(param_type,column).child(child,3);
            QModelIndex child_default_value = model->index(param_type,column).child(child,4);
            QModelIndex child_descripion = model->index(param_type,column).child(child,5);

            QString d_child_name = model->data(child_name).toString();
            QString d_child_type = model->data(child_type).toString();
            QString d_child_topic = model->data(child_topic).toString();
            QString d_child_default_value = model->data(child_default_value).toString();
            QString d_child_descripion = model->data(child_descripion).toString();
        }
    }
    */
    //Getting data from table - see https://www.qtcentre.org/threads/1294-QT4-get-content-of-a-QAbstractItemModel
    /*
    int row = 1;
    //int column = 0;
    //rows:      0 = Publishers 1 = Subscribers 2 = Parameters
    //columns:   0 = Items(rows)1 = Name        2 = Type        3 = Topic       4 = Default Value       5 = Description
    QModelIndex idx = (model->index(row, column));
    bool has_child = model->hasChildren(model->index(row,column));
    //int num_child = model->rowCount(model->index(row,column));
    QModelIndex child_index = model->index(row,column).child(2,0); //child(row,column) column: see above
    QString s;
    if (has_child)
    {
        s = model->data(child_index).toString();
    }
    else
    {
        s = "Success(not really yet)";
    }

    //This is the output after generation
    label_3->setText(s);//"Success(not really yet)");
    if (num_child > 10)
    {
        label_3->setText(QString::number(num_child));
    }
    */
    //Where files go: ~/Qt/Examples/Qt-5.11.2/widgets/itemviews/build-editabletreemodel-Desktop_Qt_5_11_2_GCC_64bit-Debug

}

/*
 *     //This function is what happens after "generate" button is pressed.
    QAbstractItemModel *model = view->model();

    //Getting data from text boxes
    for (int row = 0; row < model->rowCount(); row++)
    {
            row++;//REMOVE
    }

    //opening file
    QString node_file_name = lineEdit->text();
    std::string file_name = node_file_name.toStdString();
    //string name_space = lineEdit_2;
    //double node_file_name = lineEdit->text().toDouble(); //this worked
    //std:: ofstream outfile;
    //outfile.open(file_name);
    //outfile << "Test\n";
    //outfile.close();


    QFile file("out.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    QTextStream out(&file);
    out << "The magic number is: " << 49 << "\n";

    //Getting data from table - see https://www.qtcentre.org/threads/1294-QT4-get-content-of-a-QAbstractItemModel
    int row = 1;
    int column = 0;
    //rows:      0 = Publishers 1 = Subscribers 2 = Parameters
    //columns:   0 = Items(rows)1 = Name        2 = Type        3 = Topic       4 = Default Value       5 = Description
    QModelIndex idx = (model->index(row, column));
    QString s = model->data(idx).toString();


    //This is the output after generation
    label_3->setText("Success(not really yet)");
    //label_3->setText(node_file_name);
    //label_3->setText(s);
    //lineEdit_3->setText(QString::number(node_file_name));// = node_file_name;
    //Where files go: ~/Qt/Examples/Qt-5.11.2/widgets/itemviews/build-editabletreemodel-Desktop_Qt_5_11_2_GCC_64bit-Debug
*/
void headding(QTextStream &out, QString FILE_NAME)
{
    out << "/*" << "\n";
    out << " * " << FILE_NAME << "\n";
    out << " *" << "\n";
    out << " * " <<"DESCRIPTION_HERE"<< "\n";
    out << " * " <<"VERSION_#" << "\n";
    out << " * " <<"Last changed:" << QDate::currentDate().toString()<< "\n";
    out << " * " <<"AUTHORS" << "\n";
    out << " * " <<"MAINTAINERS" << "\n";
    out << " * " <<"MIT License" << "\n";
    out << " * " <<"Copyright (c) 2018 GOFIRST-Robotics" << "\n";
    out << " */ "  << "\n";
    out << "\n";
    out << "// ROS Libs" << "\n";
    out << "#include <ros/ros.h>" << "\n";
    out << "#include \"AUTOGEN_DEPS\"" << "\n";
    out << "TODO: // #include <pub-sub_nameN_type.h>" << "\n";
    out << "\n";
    out << ":BEGIN Custom_Libs" << "\n";
    out << "\n";
    out << "Subscribers (inputs)" << "\n";
}
void comment_subs(QTextStream &out, std::vector<pub_sub_info> subs)
{
    out << "// Subscribers (inputs)" << "\n";
    for (unsigned i =0; i < subs.size(); i++)
    {
        out << "//  " << subs[i].name <<" (" << subs[i].type << "): " << subs[i].topic << "\n" << "//       "<< subs[i].description << "\n" ;
    }
}

void comment_pubs(QTextStream &out, std::vector<pub_sub_info> pubs)
{
    out << "// Publishers (outputs)" << "\n";
    for (unsigned i =0; i < pubs.size(); i++)
    {
        out << "//  " << pubs[i].name <<" (" << pubs[i].type << "): " << pubs[i].topic << "\n" << "//       "<< pubs[i].description << "\n" ;
    }
    out << "\n";
}
void comment_params(QTextStream &out, std::vector<pub_sub_info> params)
{
    out << "// Parameters (settings)" << "\n";
    for (unsigned i =0; i < params.size(); i++)
    {
        out << "//  " << params[i].name <<" (" << params[i].type << "): default=" << params[i].default_value << "(," << params[i].description<<")\n";
    }
    out << "\n";
}
void ROS_node_and_pub(QTextStream &out, std::vector<pub_sub_info> pubs)
{
    out << "// ROS Node and Publishers" << "\n";
    out << "ros::NodeHandle * nh;" << "\n";
    for (unsigned i =0; i < pubs.size(); i++)
    {
        out << "ros::Publisher " << pubs[i].name <<"_pub;";
    }
    out << "\n";
}
void ROS_topics(QTextStream &out, std::vector<pub_sub_info> subs)
{
    out << "// ROS Topics" << "\n";
    for (unsigned i =0; i < subs.size(); i++)
    {
        out << "std::string " << subs[i].name <<"_topic = " <<subs[i].topic << ";\n";
    }
    out << "\n";
}
void ROS_callbacks(QTextStream &out, std::vector<pub_sub_info> subs)
{
    out << "// ROS Callbacks" << "\n";
    //mavros_msgs/GlobalPositionTarget

    for (unsigned i =0; i < subs.size(); i++)
    {
        QString type = subs[i].type;
        type.replace("/","::");
        out << "void " << subs[i].name <<"_callback(const " << type << "::ConstPtr& msg);\n";
    }
    out << "\n";
}
void Params(QTextStream &out, std::vector<pub_sub_info> params)
{
    out << "// ROS Params" << "\n";
    for (unsigned i =0; i < params.size(); i++)
    {
        out << params[i].type << " " << params[i].name << " = " << params[i].default_value << ";\n";
    }
    out << "\n";
}

void main_init_ROS(QTextStream &out, QString FILE_NAME)
{
    out <<"int main(int argc, char** argv){\n"
        <<" // Init ROS\n"
        <<" ros::init(argc, argv, " << FILE_NAME << "):\n"
        <<" nh = new ros::NodeHandle(\"~\")\n";
}

void main_subs(QTextStream &out, std::vector<pub_sub_info> subs)
{
    out<<"// Subscribers\n";
    for (unsigned i =0; i < subs.size(); i++)
    {
        out <<" ros::Subscriber " << subs[i].name <<"_sub = nh->subscribe(" << subs[i].topic<<", " <<subs[i].buflen << ", "<< subs[i].name <<"_callback);\n";
    }
    out << "\n";
}

void main_pubs(QTextStream &out,std::vector<pub_sub_info> pubs)
{
    out<<"// Publishers\n";
    for (unsigned i =0; i < pubs.size(); i++)
    {
        QString type = pubs[i].type;
        type.replace("/","::");
        out <<" "<< pubs[i].name <<" _pub = nh->advertise<" << type << ">("<< pubs[i].topic<< ", " <<pubs[i].buflen<<");\n";
    }
    out << "\n";
}

void main_params_spin_end(QTextStream &out,std::vector<pub_sub_info> params)
{
    out<<"  // Params\n";
    for (unsigned i =0; i < params.size(); i++)
    {
        out << "    nh->param<" << params[i].topic <<", "<<params[i].name<<", "<<params[i].default_value<<";\n";
    }
    out << "\n";
    out << "    //Spin" <<"\n";
    out << "ros::spin();" << "\n";
    out << "}\n";
}

void many_sub_callbacks(QTextStream &out, std::vector<pub_sub_info> subs)
{
    for (unsigned i =0; i < subs.size(); i++)
    {
        QString type = subs[i].type;
        type.replace("/","::");
        out << "void " << subs[i].name << "_callback(const "<< type << "::ConstPtr& msg){\n";
        out << "//:BEGIN sub_name1_callback" <<"\n\n\n";
        out <<"//:END\n}\n";
    }
    out <<"\n";
}

