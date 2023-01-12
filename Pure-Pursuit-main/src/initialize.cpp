#include "initialize.hpp"
#include "odom.hpp"
#include "main.h"
#include "config.hpp"
#include "display/lvgl.h"
#include "pros/apix.h"

int autonChoice = 1;



lv_obj_t *auto1Button;
lv_obj_t *A1BLabel;

lv_obj_t *auto2Button;
lv_obj_t *A2BLabel;

lv_obj_t *auto3Button;
lv_obj_t *A3BLabel;

lv_obj_t *auto4Button;
lv_obj_t *A4BLabel;

lv_obj_t *auto5Button;
lv_obj_t *A5BLabel;

lv_obj_t *auto6Button;
lv_obj_t *A6BLabel;

lv_obj_t *auto7Button;
lv_obj_t *A7BLabel;

lv_obj_t *auto8Button;
lv_obj_t *A8BLabel;

lv_obj_t *auto9Button;
lv_obj_t *A9BLabel;

lv_obj_t *AutonChoice;
lv_obj_t *allianceButton;
lv_obj_t *allianceButtonLabel;

lv_obj_t *OdomDisplay;
lv_obj_t *mainDisplay;
lv_obj_t *AutonDisplay;
lv_obj_t *autonSelectedDisplay;
lv_obj_t *infoDisplay;
lv_obj_t *infoDisplay2;
lv_obj_t *twotenDisplay;

lv_style_t myButtonStyleREL; // relesed style
lv_style_t myButtonStylePR;	 // pressed style

lv_style_t directorStyleREL; // relesed style
lv_style_t directorStylePR;	 // pressed style

// Directors
lv_obj_t *homeDirector;
lv_obj_t *homeDirectorLabel;
lv_obj_t *autonSelectorDirector;
lv_obj_t *autonSelectorDirectorLabel;
lv_obj_t *informationDirector;
lv_obj_t *informationDirectorLabel;
lv_obj_t *gifDirector;
lv_obj_t *gifDirectorLabel;
lv_style_t gifStyleREL; // relesed style
lv_style_t gifStylePR;	// pressed style

lv_obj_t *homePage = lv_page_create(lv_scr_act(), NULL);
lv_obj_t *autoPage = lv_page_create(lv_scr_act(), NULL);
lv_obj_t *infoPage = lv_page_create(lv_scr_act(), NULL);
lv_obj_t *gifPage = lv_page_create(lv_scr_act(), NULL);

bool redAlliance = true; // 1 == red. 0 == blue

static lv_res_t btn_click_action(lv_obj_t *btn)
{
	uint8_t id = lv_obj_get_free_num(btn); // id usefull when there are multiple buttons
	if (id == 101)						   // home director
	{
		lv_obj_set_hidden(homePage, false);
		lv_obj_set_hidden(autoPage, true);
		lv_obj_set_hidden(infoPage, true);
		lv_obj_set_hidden(gifPage, true);
	}
	else if (id == 102) // auton director
	{
		lv_obj_set_hidden(homePage, true);
		lv_obj_set_hidden(autoPage, false);
		lv_obj_set_hidden(infoPage, true);
		lv_obj_set_hidden(gifPage, true);
	}
	else if (id == 103) // information director
	{
		lv_obj_set_hidden(homePage, true);
		lv_obj_set_hidden(autoPage, true);
		lv_obj_set_hidden(infoPage, false);
		lv_obj_set_hidden(gifPage, true);
	}
	else if (id == 104) // gif
	{
		lv_obj_set_hidden(homePage, true);
		lv_obj_set_hidden(autoPage, true);
		lv_obj_set_hidden(infoPage, true);
		lv_obj_set_hidden(gifPage, false);
	}

	// auton selector
	if (id == 0)
	{
		char buffer[100];
		sprintf(buffer, "Auton 1 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 1;
	}
	else if (id == 1)
	{
		char buffer[100];
		sprintf(buffer, "Auton 2 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 2;
	}
	else if (id == 2)
	{
		char buffer[100];
		sprintf(buffer, "Auton 3 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 3;
	}
	else if (id == 3)
	{
		char buffer[100];
		sprintf(buffer, "Auton 4 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 4;
	}
	else if (id == 4)
	{
		char buffer[100];
		sprintf(buffer, "Auton 5 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 5;
	}
	else if (id == 5)
	{
		char buffer[100];
		sprintf(buffer, "Auton 6 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 6;
	}
	else if (id == 6)
	{
		char buffer[100];
		sprintf(buffer, "Auton 7 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 7;
	}
	else if (id == 7)
	{
		char buffer[100];
		sprintf(buffer, "Auton 8 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 8;
	}
	else if (id == 8)
	{
		char buffer[100];
		sprintf(buffer, "Auton 9 Selected");
		lv_label_set_text(AutonChoice, buffer);
		autonChoice = 9;
	}

	if (id == 10) // alliance switch
	{
		if (redAlliance)
		{
			char buffer[10];
			sprintf(buffer, "Blue");
			lv_label_set_text(allianceButtonLabel, buffer);
			lv_btn_set_style(auto1Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto2Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto3Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto4Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto5Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto6Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto7Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto8Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(auto9Button, LV_BTN_STYLE_REL, &myButtonStylePR);
			lv_btn_set_style(allianceButton, LV_BTN_STYLE_REL, &myButtonStylePR); // set the relesed style
			lv_btn_set_style(allianceButton, LV_BTN_STYLE_PR, &myButtonStyleREL); // set the pressed style

			redAlliance = false;
		}
		else
		{
			char buffer[10];
			sprintf(buffer, "Red");
			lv_label_set_text(allianceButtonLabel, buffer);
			lv_btn_set_style(auto1Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto2Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto3Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto4Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto5Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto6Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto7Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto8Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(auto9Button, LV_BTN_STYLE_REL, &myButtonStyleREL);
			lv_btn_set_style(allianceButton, LV_BTN_STYLE_REL, &myButtonStyleREL); // set the relesed style
			lv_btn_set_style(allianceButton, LV_BTN_STYLE_PR, &myButtonStylePR);   // set the pressed style
			redAlliance = true;
		}
	}

	// Auton Display
	if (autonChoice == 0)
	{
		lv_label_set_text(autonSelectedDisplay, "No Auton Selected"); // set
	}
	else
	{
		if (redAlliance)
		{
			char buffer[1];
			sprintf(buffer, "Auton: RED %d", autonChoice);
			lv_label_set_text(autonSelectedDisplay, buffer); // set
		}
		else if (!redAlliance)
		{
			char buffer[1];
			sprintf(buffer, "Auton: BLUE %d", autonChoice);
			lv_label_set_text(autonSelectedDisplay, buffer); // set
		}
	}

	return LV_RES_OK;
}
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initialize()
{

	// PAGES
	lv_page_set_sb_mode(homePage, LV_SB_MODE_OFF); // scroll bar
	lv_page_set_sb_mode(autoPage, LV_SB_MODE_OFF); // scroll bar
	lv_page_set_sb_mode(infoPage, LV_SB_MODE_OFF); // scroll bar
	lv_page_set_sb_mode(gifPage, LV_SB_MODE_OFF);  // scroll bar

	lv_obj_set_size(homePage, 450, 200);
	lv_obj_align(homePage, NULL, LV_ALIGN_CENTER, 0, -30);

	lv_obj_set_size(autoPage, 450, 200);
	lv_obj_align(autoPage, NULL, LV_ALIGN_CENTER, 0, -30);

	lv_obj_set_size(infoPage, 450, 200);
	lv_obj_align(infoPage, NULL, LV_ALIGN_CENTER, 0, -30);

	lv_obj_set_size(gifPage, 450, 200);
	lv_obj_align(gifPage, NULL, LV_ALIGN_CENTER, 0, -30);

	// STYLES
	lv_style_copy(&directorStyleREL, &lv_style_plain);
	directorStyleREL.body.main_color = LV_COLOR_MAKE(249, 43, 43);
	directorStyleREL.body.grad_color = LV_COLOR_MAKE(249, 43, 43);
	directorStyleREL.body.radius = 0;
	directorStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

	lv_style_copy(&directorStylePR, &lv_style_plain);
	directorStylePR.body.main_color = LV_COLOR_MAKE(0, 0, 0);
	directorStylePR.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
	directorStylePR.body.radius = 0;
	directorStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

	lv_style_copy(&gifStyleREL, &lv_style_plain);
	gifStyleREL.body.main_color = LV_COLOR_MAKE(183, 159, 0);
	gifStyleREL.body.grad_color = LV_COLOR_MAKE(183, 159, 0);
	gifStyleREL.body.radius = 0;
	gifStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

	lv_style_copy(&myButtonStyleREL, &lv_style_plain);
	myButtonStyleREL.body.main_color = LV_COLOR_MAKE(249, 43, 43);
	myButtonStyleREL.body.grad_color = LV_COLOR_MAKE(249, 43, 43);
	myButtonStyleREL.body.radius = 0;
	myButtonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

	lv_style_copy(&myButtonStylePR, &lv_style_plain);
	myButtonStylePR.body.main_color = LV_COLOR_MAKE(43, 105, 249);
	myButtonStylePR.body.grad_color = LV_COLOR_MAKE(43, 105, 249);
	myButtonStylePR.body.radius = 0;
	myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

	// DIRECTORS
	homeDirector = lv_btn_create(lv_scr_act(), NULL);						// create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(homeDirector, 101);									// set button iD to 0
	lv_btn_set_action(homeDirector, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(homeDirector, LV_BTN_STYLE_REL, &directorStyleREL);	// set the released style
	lv_btn_set_style(homeDirector, LV_BTN_STYLE_PR, &directorStylePR);		// set the pressed style
	lv_obj_set_size(homeDirector, 140, 50);									// set the button size
	lv_obj_align(homeDirector, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);		// set the position to top mid

	homeDirectorLabel = lv_label_create(homeDirector, NULL); // create label and puts it inside of the button
	lv_label_set_text(homeDirectorLabel, "Home");			 // sets label text

	autonSelectorDirector = lv_btn_create(lv_scr_act(), NULL);						 // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(autonSelectorDirector, 102);								 // set button iD to 0
	lv_btn_set_action(autonSelectorDirector, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(autonSelectorDirector, LV_BTN_STYLE_REL, &directorStyleREL);	 // set the released style
	lv_btn_set_style(autonSelectorDirector, LV_BTN_STYLE_PR, &directorStylePR);		 // set the pressed style
	lv_obj_set_size(autonSelectorDirector, 140, 50);								 // set the button size
	lv_obj_align(autonSelectorDirector, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 140, 0);		 // set the position to top mid

	autonSelectorDirectorLabel = lv_label_create(autonSelectorDirector, NULL); // create label and puts it inside of the button
	lv_label_set_text(autonSelectorDirectorLabel, "Auton Select");			   // sets label text

	informationDirector = lv_btn_create(lv_scr_act(), NULL);					   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(informationDirector, 103);								   // set button iD to 0
	lv_btn_set_action(informationDirector, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(informationDirector, LV_BTN_STYLE_REL, &directorStyleREL);	   // set the released style
	lv_btn_set_style(informationDirector, LV_BTN_STYLE_PR, &directorStylePR);	   // set the pressed style
	lv_obj_set_size(informationDirector, 140, 50);								   // set the button size
	lv_obj_align(informationDirector, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 280, 0);	   // set the position to top mid

	informationDirectorLabel = lv_label_create(informationDirector, NULL); // create label and puts it inside of the button
	lv_label_set_text(informationDirectorLabel, "Info");				   // sets label text

	gifDirector = lv_btn_create(lv_scr_act(), NULL);					   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(gifDirector, 104);								   // set button iD
	lv_btn_set_action(gifDirector, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(gifDirector, LV_BTN_STYLE_REL, &gifStyleREL);		   // set the released style
	lv_btn_set_style(gifDirector, LV_BTN_STYLE_PR, &directorStylePR);	   // set the pressed style
	lv_obj_set_size(gifDirector, 60, 50);								   // set the button size
	lv_obj_align(gifDirector, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 420, 0);	   // set the position to top mid

	gifDirectorLabel = lv_label_create(gifDirector, NULL); // create label and puts it inside of the button
	lv_label_set_text(gifDirectorLabel, "210Y");		   // sets label text

	// HomePage
	OdomDisplay = lv_label_create(homePage, NULL);
	lv_obj_align(OdomDisplay, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0); // set the position to mid

	AutonDisplay = lv_label_create(homePage, NULL);
	lv_obj_align(AutonDisplay, NULL, LV_ALIGN_IN_LEFT_MID, 100, 0); // set the position to mid

	autonSelectedDisplay = lv_label_create(homePage, NULL);					// create label and puts it inside of the button
	lv_obj_align(autonSelectedDisplay, NULL, LV_ALIGN_IN_LEFT_MID, 0, 150); // set the position to mid
	lv_label_set_text(autonSelectedDisplay, "No Auton Selected");			// set

	mainDisplay = lv_label_create(homePage, NULL);
	lv_obj_align(mainDisplay, NULL, LV_ALIGN_IN_RIGHT_MID, -150, -70); // set the position to mid

	// AutoPage
	auto1Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto1Button, 0);								   // set button iD to 0
	lv_btn_set_action(auto1Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto1Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the released style
	lv_btn_set_style(auto1Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto1Button, 100, 50);								   // set the button size
	lv_obj_align(auto1Button, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);		   // set the position to top mid

	A1BLabel = lv_label_create(auto1Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A1BLabel, "Auton 1");		   // sets label text

	auto2Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto2Button, 1);								   // set button iD to 0
	lv_btn_set_action(auto2Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto2Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto2Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto2Button, 100, 50);								   // set the button size
	lv_obj_align(auto2Button, NULL, LV_ALIGN_IN_TOP_LEFT, 105, 0);		   // set the position to top mid

	A2BLabel = lv_label_create(auto2Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A2BLabel, "Auton 2");		   // sets label text

	auto3Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto3Button, 2);								   // set button iD to 0
	lv_btn_set_action(auto3Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto3Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto3Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto3Button, 100, 50);								   // set the button size
	lv_obj_align(auto3Button, NULL, LV_ALIGN_IN_TOP_LEFT, 210, 0);		   // set the position to top mid

	A3BLabel = lv_label_create(auto3Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A3BLabel, "Auton 3");		   // sets label text

	auto4Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto4Button, 3);								   // set button iD to 0
	lv_btn_set_action(auto4Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto4Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto4Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto4Button, 100, 50);								   // set the button size
	lv_obj_align(auto4Button, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 55);		   // set the position to top mid

	A4BLabel = lv_label_create(auto4Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A4BLabel, "Auton 4");		   // sets label text

	auto5Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto5Button, 4);								   // set button iD to 0
	lv_btn_set_action(auto5Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto5Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto5Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto5Button, 100, 50);								   // set the button size
	lv_obj_align(auto5Button, NULL, LV_ALIGN_IN_TOP_LEFT, 105, 55);		   // set the position to top mid

	A5BLabel = lv_label_create(auto5Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A5BLabel, "Auton 5");		   // sets label text

	auto6Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto6Button, 5);								   // set button iD to 0
	lv_btn_set_action(auto6Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto6Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto6Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto6Button, 100, 50);								   // set the button size
	lv_obj_align(auto6Button, NULL, LV_ALIGN_IN_TOP_LEFT, 210, 55);		   // set the position to top mid

	A6BLabel = lv_label_create(auto6Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A6BLabel, "Auton 6");		   // sets label text

	auto7Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto7Button, 6);								   // set button iD to 0
	lv_btn_set_action(auto7Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto7Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto7Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto7Button, 100, 50);								   // set the button size
	lv_obj_align(auto7Button, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 110);		   // set the position to top mid

	A7BLabel = lv_label_create(auto7Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A7BLabel, "Auton 7");		   // sets label text

	auto8Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto8Button, 7);								   // set button iD to 0
	lv_btn_set_action(auto8Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto8Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto8Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto8Button, 100, 50);								   // set the button size
	lv_obj_align(auto8Button, NULL, LV_ALIGN_IN_TOP_LEFT, 105, 110);	   // set the position to top mid

	A8BLabel = lv_label_create(auto8Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A8BLabel, "Auton 8");		   // sets label text

	auto9Button = lv_btn_create(autoPage, NULL);						   // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(auto9Button, 8);								   // set button iD to 0
	lv_btn_set_action(auto9Button, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(auto9Button, LV_BTN_STYLE_REL, &myButtonStyleREL);	   // set the relesed style
	lv_btn_set_style(auto9Button, LV_BTN_STYLE_PR, &myButtonStylePR);	   // set the pressed style
	lv_obj_set_size(auto9Button, 100, 50);								   // set the button size
	lv_obj_align(auto9Button, NULL, LV_ALIGN_IN_TOP_LEFT, 210, 110);	   // set the position to top mid

	A9BLabel = lv_label_create(auto9Button, NULL); // create label and puts it inside of the button
	lv_label_set_text(A9BLabel, "Auton 9");		   // sets label text

	AutonChoice = lv_label_create(autoPage, NULL);			 // create label and puts it on the screen
	lv_label_set_text(AutonChoice, "No Auton Selected");	 // sets label text
	lv_obj_align(AutonChoice, NULL, LV_ALIGN_CENTER, 0, 90); // set the position to mid

	allianceButton = lv_btn_create(autoPage, NULL);							  // create button, lv_scr_act() is deafult screen object
	lv_obj_set_free_num(allianceButton, 10);								  // set button iD to 0
	lv_btn_set_action(allianceButton, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
	lv_btn_set_style(allianceButton, LV_BTN_STYLE_REL, &myButtonStyleREL);	  // set the relesed style
	lv_btn_set_style(allianceButton, LV_BTN_STYLE_PR, &myButtonStylePR);	  // set the pressed style
	lv_obj_set_size(allianceButton, 100, 50);								  // set the button size
	lv_obj_align(allianceButton, NULL, LV_ALIGN_IN_TOP_LEFT, 315, 55);		  // set the position to top mid

	allianceButtonLabel = lv_label_create(allianceButton, NULL); // create label and puts it inside of the button
	lv_label_set_text(allianceButtonLabel, "RED");				 // sets label text

	// InfoPage
	infoDisplay = lv_label_create(infoPage, NULL);
	lv_obj_align(infoDisplay, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0); // set the position to mid
	lv_label_set_text(infoDisplay, "ODOM INFORMATION");			 // set

	infoDisplay2 = lv_label_create(infoPage, NULL);
	lv_obj_align(infoDisplay2, NULL, LV_ALIGN_IN_RIGHT_MID, -100, 0); // set the position to mid
	lv_label_set_text(infoDisplay2, "GPS INFORMATION");				  // set

	lv_obj_set_hidden(homePage, false);
	lv_obj_set_hidden(autoPage, true);
	lv_obj_set_hidden(infoPage, true);
	lv_obj_set_hidden(gifPage, true);

	// gifPage
	twotenDisplay = lv_label_create(gifPage, NULL);
	lv_obj_align(gifPage, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_label_set_text(twotenDisplay, "Prepare for Trouble\n And Make It Double");

	//     ."`".
	// .-./ _=_ \.-.
	// {  (,(oYo),) }}
	// {{ |   "   |} }
	// { { \(---)/  }}
	// {{  }'-=-'{ } }
	// { { }._:_.{  }}
	// {{  } -:- { } }
	// jgs   {_{ }`===`{  _}
	// ((((\)     (/))))
	// Set Rotation Sensors

	// set6MDrive();
	setBreakTypeCoast();
	// clamp.set_value(1); // opens
	// grabber.set_value(1); // closes
	// tilter.set_value(0);
  	// tilter2.set_value(1);
	openClamp();
	tiltDown();
	setGrab(true);

	liftPot.reset();
	liftPot.reset_position();
	leftRotation.set_data_rate(10);
	rightRotation.set_data_rate(10);
	centerRotation.set_data_rate(10);
	fwdRotation.set_data_rate(10);
	leftRotation.reset();
	rightRotation.reset();
	centerRotation.reset();
	fwdRotation.reset();
	leftRotation.reset_position();
	rightRotation.reset_position();
	centerRotation.reset_position();
	fwdRotation.reset_position();
	leftRotation.set_reversed(true);
	rightRotation.set_reversed(true);
	centerRotation.set_reversed(true);
	fwdRotation.set_reversed(false);

	// Reset inertials
	imuLeft.reset();
	imuRight.reset();
	while (imuLeft.get_rotation() == PROS_ERR_F || imuRight.get_rotation() == PROS_ERR_F)
	{

		pros::delay(100);
	}
	pros::delay(1500);
	if (imuLeft.get_rotation() != PROS_ERR_F && imuRight.get_rotation() != PROS_ERR_F)
		lv_label_set_text(mainDisplay, "Initialized");

	master.rumble(".");
	// Set Breaktypes

	lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// LCD
	if (isnanf(pos_x))
	{
		pos_x = 0;
	}
	if (isnanf(pos_y))
	{
		pos_y = 0;
	}
	lvgldisplay();

		// std::cout<<"CUM"<<std::endl;
}

void disabled() {
	// std::cout<<"disabled";
	// while(1){
	// 	lvgldisplay();
	// 	// std::cout<<"CUM"<<std::end
	// 	odom(); counter++;
	// 	pros::delay(10);
	// }

}

void competition_initialize() {

}
