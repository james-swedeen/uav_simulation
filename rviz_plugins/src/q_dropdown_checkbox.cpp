/**
 * @File: qt_dropdown_checkbox.cpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Implementation file for custom Qt combo box with dropdown
 * checkboxes.
 *
 * @details
 * This framework is adapted from an example by Khisamutdinov Radik
 * that can be found at:
 * https://notes84.blogspot.com/2016/05/c-qt56-combobox-with-checkboxes.html?m=0
 */

// Local Headers
#include "rviz_plugins/gui_tools/q_dropdown_checkbox.hpp"

// C++ Headers
#include <iostream>
#include <vector>

// Qt GUI Headers
#include <QtWidgets>

namespace rviz_plugins{
namespace gui_tools{

/**
 * @brief Construct a new QDropdownCheckbox object
 *
 * @details Initialize member variables. Set default text
 * that will appear when widget is rendered. Connect signals
 * and slots.
 *
 * @param parent: Parent widget that this widget belongs to
 */
QDropdownCheckbox::QDropdownCheckbox(QWidget *parent) : QComboBox(parent){
  // Allow user to change the state of the dropdown box contents
  this->setEditable(true);

  // Initialize private members
  this->m_current_text_ = "";
  this->m_default_text_ = "";
  this->m_model_ = new QStandardItemModel;

  // Set initial text and make read only so user can't change anything but checkboxes
  this->lineEdit()->setText(this->m_current_text_);
  this->lineEdit()->setReadOnly(true);

  // Update changes visually
  this->update_textbox();

  // When any item changes in the dropdown box, the update_text function will be called
  connect(this->m_model_, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(update_textbox()));
}

/* === CHECKBOX CREATION === */
/**
 * @brief Add a checkbox with a tooltip
 *
 * @details Adds a checkbox to the item model maintained by the class.
 * The checkbox is given a specified label and tooltip. The tooltip will
 * appear in the GUI when hovering over any region the checkbox widget
 * occupies. The tooltip is meant to provide additional information about
 * what effects the checkbox might have without needing to refer to
 * documentation.
 *
 * @param checkbox_label: The text displayed next to the checkbox
 * @param tooltip: Useful information about the checkbox's purpose
 */
void QDropdownCheckbox::add_checkbox(const QString &checkbox_label, const QString &tooltip){
  auto row = this->m_model_->rowCount();

  // Create new checkbox item
  QStandardItem *checkbox = new QStandardItem;
  checkbox->setText(checkbox_label);

  // Set flags such that item becomes an actual checkbox
  checkbox->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
  checkbox->setData(Qt::Unchecked, Qt::CheckStateRole);

  // Add tooltip
  if (tooltip.compare("") != 0){
    checkbox->setToolTip(tooltip);
  }

  // Update model with new item, then update model to include new widgets
  this->m_model_->setItem(row, checkbox);
  this->setModel(this->m_model_);
}

/**
 * @brief Add multiple checkboxes with labels
 *
 * @details Use the add_checkbox function with all the labels in the
 * provided list. No tooltips are added this way
 *
 * @param checkbox_labels: List of labels for checkboxes to add
 */
void QDropdownCheckbox::add_checkboxes(const QStringList &checkbox_labels){
  // Iterate over each label and add a checkbox for it
  for (int s = 0; s < checkbox_labels.size(); s++){
    this->add_checkbox(checkbox_labels[s]);
  }
}

/**
 * @brief Add multiple checkboxes with labels and tooltips
 *
 * @details Use the add_checkbox function with all the labels in the
 * provided list, along with associated tooltips
 *
 * @param checkbox_descriptions: Vector of (label, tooltip) pairs for new checkboxes
 */
void QDropdownCheckbox::add_checkboxes(const std::vector<std::pair<QString, QString>> &checkbox_descriptions){
  // Iterate over each label and add a checkbox and tooltip for it
  for (size_t s = 0; s < checkbox_descriptions.size(); s++){
    this->add_checkbox(checkbox_descriptions[s].first, checkbox_descriptions[s].second);
  }
}

/* === GETTERS AND SETTERS === */
/**
 * @brief Get the status of the currently checked boxes in a vector
 *
 * @return std::vector<bool>
 */
std::vector<bool> QDropdownCheckbox::get_checked(){
  std::vector<bool> checked_idxs;

  // For each checked item, add the index of that item to the array
  for (int i = 0; i < this->m_model_->rowCount(); i++){
    checked_idxs.emplace_back(this->m_model_->item(i)->checkState() == Qt::Checked);
  }

  return checked_idxs;
}

/**
 * @brief Get the current number of checkboxes in the list
 *
 * @return size_t
 */
int QDropdownCheckbox::get_num_checkboxes(){
  return this->m_model_->rowCount();
}

/**
 * @brief Set the textbox object
 *
 * @details Should be used by calling code as an initializer upon
 * dropdown checkbox creation
 */
void QDropdownCheckbox::set_marked_idxs(const std::vector<int> &checkbox_idxs){
  // From list of checkbox indices, mark selected as checked
  if (!checkbox_idxs.empty()){
    for (auto c_it = checkbox_idxs.begin(); c_it != checkbox_idxs.end(); ++c_it){
      this->m_model_->item(*c_it)->setData(Qt::Checked, Qt::CheckStateRole);
    }
  }

  // Update text in textbox now
  this->update_textbox();
}


/**
 * @brief Set the default text to appear when no items are selected
 *
 * @param default_text: Text to appear
 */
void QDropdownCheckbox::set_default_text(const QString &default_text){
  this->m_default_text_ = default_text;
  this->update_textbox();
}

/* === SLOTS === */
/**
 * @brief Updates text in dropdown textbox
 *
 * @details Creates in-order string of all checked items
 * to display in the textbox at the top of the combo box.
 * This will trigger after any item in the dropdown box is changed
 */
void QDropdownCheckbox::update_textbox(){
  this->m_current_text_ = "";
  auto rows = this->m_model_->rowCount();

  // Get number of checked items
  int num_checked = 0;
  for (int i = 0; i < rows; i++){
    if (this->m_model_->item(i)->checkState() == Qt::Checked){
      num_checked++;
    }
  }

  // As long as one box is checked, update text with number of items checked
  if (num_checked != 0){
    this->m_current_text_ =
      QString::fromStdString(std::to_string(num_checked) + " selected");
    this->lineEdit()->setStyleSheet("");  // Set default settings for appearance
  } else {
    // Otherwise, display greyed out hint text
    this->m_current_text_ = this->m_default_text_;
    this->lineEdit()->setStyleSheet("color: grey");
  }

  // Override default selection behavior so text displays properly
  this->setCurrentIndex(-1);

  // Set new text
  this->lineEdit()->setText(this->m_current_text_);
}

/* === EVENT HANDLERS === */
/**
 * @brief Ignore any mouse-wheel events
 *
 * @details Override the inherited function and
 * ignore the event entirely. Without this, the textbox
 * display text would act inconsistently
 *
 * @param e: Event to ignore
 */
void QDropdownCheckbox::wheelEvent(QWheelEvent *e) {
  e->ignore();
}
}  // namespace gui_tools
}  // namespace rviz_plugins
