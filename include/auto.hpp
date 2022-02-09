#ifndef _AUTO_HPP_
#define _AUTO_HPP_

extern int chassis_step;
extern int mogo_state;
extern int arm_state;
extern int intake_state;
extern bool claw_state;
extern int auto_step;
extern int auto_part;

void change_auto_step();
void change_auto_part();

void set_up_auto();

void Skills();

bool SkillsPt1();
bool SkillsPt2();
bool SkillsPt3();
bool SkillsPt4();
bool SkillsPt5();
bool SkillsPt6();

#endif
