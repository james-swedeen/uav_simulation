/**
 * @File: qt_dropdown_checkbox.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Header file for custom Qt combo box with dropdown
 * checkboxes.
 *
 * @details
 * This framework is adapted from an example by Khisamutdinov Radik
 * that can be found at:
 * https://notes84.blogspot.com/2016/05/c-qt56-combobox-with-checkboxes.html?m=0
 */

#ifndef RVIZ_PLUGINS__GUI_TOOLS__Q_DROPDOWN_CHECKBOX_HPP_
#define RVIZ_PLUGINS__GUI_TOOLS__Q_DROPDOWN_CHECKBOX_HPP_

// C++ Headers
#include <vector>
#include <utility>

// QT GUI Headers
#include <QtWidgets>

namespace rviz_plugins{
namespace gui_tools{

class QDropdownCheckbox : public QComboBox {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT

  public:
    /**
     * @brief Construct a new QDropdownCheckbox object
     *
     * @details Initialize member variables. Set default text
     * that will appear when widget is rendered. Connect signals
     * and slots.
     *
     * @param parent: Parent widget that this widget belongs to
     */
    explicit QDropdownCheckbox(QWidget *parent = 0);

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
    void add_checkbox(const QString &checkbox_label, const QString &tooltip = "");

    /**
     * @brief Add multiple checkboxes with labels
     *
     * @details Use the add_checkbox function with all the labels in the
     * provided list. No tooltips are added this way
     *
     * @param checkbox_labels: List of labels for checkboxes to add
     */
    void add_checkboxes(const QStringList &checkbox_labels);

    /**
     * @brief Add multiple checkboxes with labels and tooltips
     *
     * @details Use the add_checkbox function with all the labels in the
     * provided list, along with associated tooltips
     *
     * @param checkbox_descriptions: Vector of (label, tooltip) pairs for new checkboxes
     */
    void add_checkboxes(const std::vector<std::pair<QString, QString>>& checkbox_descriptions);

    /* === GETTERS AND SETTERS === */
    /**
     * @brief Get the status of the currently checked boxes in a vector
     *
     * @return std::vector<bool>
     */
    std::vector<bool> get_checked();

    /**
     * @brief Get the current number of checkboxes in the list
     *
     * @return size_t
     */
    int get_num_checkboxes();

    /**
     * @brief Set the textbox object
     *
     * @details Should be used by calling code as an initializer upon
     * dropdown checkbox creation
     */
    void set_marked_idxs(const std::vector<int> &checkbox_idxs = {});

    /**
     * @brief Set the default text to appear when no items are selected
     *
     * @param default_text: Text to appear
     */
    void set_default_text(const QString &default_text);

  protected Q_SLOTS:
    /* === SLOTS === */
    /**
     * @brief Updates text in dropdown textbox
     *
     * @details Creates in-order string of all checked items
     * to display in the textbox at the top of the combo box.
     * This will trigger after any item in the dropdown box is changed
     */
    void update_textbox();

  private:
    QStandardItemModel *m_model_;   // Container for all the checkboxes
    QString m_current_text_;        // Text to display in the box when checks are present
    QString m_default_text_;        // Text to display when there is nothing checked

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
    void wheelEvent(QWheelEvent *e) override;
};

}  // end namespace gui_tools
}  // end namespace rviz_plugins
#endif  // RVIZ_PLUGINS__GUI_TOOLS__Q_DROPDOWN_CHECKBOX_HPP_
