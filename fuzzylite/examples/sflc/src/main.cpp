
#include "fl/Headers.h"

int main(int argc, char **argv)
{
    using namespace fl;

    const float INPUT_NL_MIN = -3.15;
    const float INPUT_NL_MED = -3.0;
    const float INPUT_NL_MAX = -2.8;
    const float INPUT_NM_MIN = -2.8;
    const float INPUT_NM_MED = -1.9;
    const float INPUT_NM_MAX = -1.1;
    const float INPUT_N_MIN = -1.1;
    const float INPUT_N_MED = -0.9;
    const float INPUT_N_MAX = -0.6;
    const float INPUT_NS_MIN = -0.6;
    const float INPUT_NS_MED = -0.5;
    const float INPUT_NS_MAX = -0.4;
    const float INPUT_ZN_MIN = -0.4;
    const float INPUT_ZN_MED = -0.25;
    const float INPUT_ZN_MAX = -0.1;
    const float INPUT_Z_MIN = -0.1; 
    const float INPUT_Z_MED = 0; //-------------
    const float INPUT_Z_MAX = 0.1;
    const float INPUT_ZP_MIN = 0.1;
    const float INPUT_ZP_MED = 0.25;
    const float INPUT_ZP_MAX = 0.4;
    const float INPUT_PS_MIN = 0.4;
    const float INPUT_PS_MED = 0.5;
    const float INPUT_PS_MAX = 0.6;
    const float INPUT_P_MIN = 0.6;
    const float INPUT_P_MED = 0.9;
    const float INPUT_P_MAX = 1.1;
    const float INPUT_PM_MIN = 1.1;
    const float INPUT_PM_MED = 1.9;
    const float INPUT_PM_MAX = 2.8;
    const float INPUT_PL_MIN = 2.8;
    const float INPUT_PL_MED = 3.0;
    const float INPUT_PL_MAX = 3.15;

    const float OUTPUT_ANG_NL_MIN = -1;
    const float OUTPUT_ANG_NL_MED = -0.8;
    const float OUTPUT_ANG_NL_MAX = -0.6;
    const float OUTPUT_ANG_NM_MIN = -0.6;
    const float OUTPUT_ANG_NM_MED = -0.45;
    const float OUTPUT_ANG_NM_MAX = -0.31;
    const float OUTPUT_ANG_N_MIN = -0.31;
    const float OUTPUT_ANG_N_MED = -0.25;
    const float OUTPUT_ANG_N_MAX = -0.19;
    const float OUTPUT_ANG_NS_MIN = -0.19;
    const float OUTPUT_ANG_NS_MED = -0.15;
    const float OUTPUT_ANG_NS_MAX = -0.1;
    const float OUTPUT_ANG_ZN_MIN = -0.1;
    const float OUTPUT_ANG_ZN_MED = -0.05;
    const float OUTPUT_ANG_ZN_MAX = -0.025;
    const float OUTPUT_ANG_Z_MIN = -0.025; 
    const float OUTPUT_ANG_Z_MED = 0; //-------------
    const float OUTPUT_ANG_Z_MAX = 0.025;
    const float OUTPUT_ANG_ZP_MIN = 0.025;
    const float OUTPUT_ANG_ZP_MED = 0.05;
    const float OUTPUT_ANG_ZP_MAX = 0.1;
    const float OUTPUT_ANG_PS_MIN = 0.1;
    const float OUTPUT_ANG_PS_MED = 0.15;
    const float OUTPUT_ANG_PS_MAX = 0.19;
    const float OUTPUT_ANG_P_MIN = 0.19;
    const float OUTPUT_ANG_P_MED = 0.25;
    const float OUTPUT_ANG_P_MAX = 0.31;
    const float OUTPUT_ANG_PM_MIN = 0.31;
    const float OUTPUT_ANG_PM_MED = 0.45;
    const float OUTPUT_ANG_PM_MAX = 0.6;
    const float OUTPUT_ANG_PL_MIN = 0.6;
    const float OUTPUT_ANG_PL_MED = 0.8;
    const float OUTPUT_ANG_PL_MAX = 1;

    Engine *engine = new Engine;
    engine->setName("SFLC");
    engine->setDescription("");

    InputVariable* Uao_gtg = new InputVariable;
    Uao_gtg->setName("Uao_gtg");
    Uao_gtg->setDescription("");
    Uao_gtg->setEnabled(true);
    Uao_gtg->setRange(-3.8,3.8); 
    Uao_gtg->setLockValueInRange(false);
    Uao_gtg->addTerm(new Triangle("NL", INPUT_NL_MIN, INPUT_NL_MED, INPUT_NL_MAX));
    Uao_gtg->addTerm(new Triangle("NM", INPUT_NM_MIN, INPUT_NM_MED, INPUT_NM_MAX));
    Uao_gtg->addTerm(new Triangle("N",  INPUT_N_MIN, INPUT_N_MED, INPUT_N_MAX));
    Uao_gtg->addTerm(new Triangle("NS", INPUT_NS_MIN, INPUT_NS_MED, INPUT_NS_MAX));
    Uao_gtg->addTerm(new Triangle("ZN", INPUT_ZN_MIN, INPUT_ZN_MED, INPUT_ZN_MAX));
    Uao_gtg->addTerm(new Triangle("Z",  INPUT_Z_MIN, INPUT_Z_MED, INPUT_Z_MAX)); //----------
    Uao_gtg->addTerm(new Triangle("ZP", INPUT_ZP_MIN, INPUT_ZP_MED, INPUT_ZP_MAX));
    Uao_gtg->addTerm(new Triangle("PS", INPUT_PS_MIN, INPUT_PS_MED, INPUT_PS_MAX));
    Uao_gtg->addTerm(new Triangle("P",  INPUT_P_MIN, INPUT_P_MED, INPUT_P_MAX));
    Uao_gtg->addTerm(new Triangle("PM", INPUT_PM_MIN, INPUT_PM_MED, INPUT_PM_MAX));
    Uao_gtg->addTerm(new Triangle("PL", INPUT_PL_MIN, INPUT_PL_MED, INPUT_PL_MAX));
    engine->addInputVariable(Uao_gtg);

    OutputVariable* linear_velocity = new OutputVariable;
    linear_velocity->setName("linear_velocity");
    linear_velocity->setDescription("");
    linear_velocity->setEnabled(true);
    linear_velocity->setRange(0, 0.7);
    linear_velocity->setLockValueInRange(false);
    linear_velocity->setAggregation(new Maximum);
    linear_velocity->setDefuzzifier(new LargestOfMaximum);
    linear_velocity->setDefaultValue(fl::nan);
    linear_velocity->setLockPreviousValue(false);
    linear_velocity->addTerm(new Triangle("S", 0, 0.11, 0.22));
    linear_velocity->addTerm(new Triangle("M", 0.19, 0.2, 0.4));
    linear_velocity->addTerm(new Triangle("L", 0.35,0.45,0.55));
    linear_velocity->addTerm(new Triangle("VL", 0.5, 0.6, 0.7));
    engine->addOutputVariable(linear_velocity);

    OutputVariable* angular_velocity = new OutputVariable;
    angular_velocity->setName("angular_velocity");
    angular_velocity->setDescription("");
    angular_velocity->setEnabled(true);
    angular_velocity->setRange(-1,1);
    angular_velocity->setLockValueInRange(false);
    angular_velocity->setAggregation(new Maximum);
    angular_velocity->setDefuzzifier(new LargestOfMaximum);
    angular_velocity->setDefaultValue(fl::nan);
    angular_velocity->setLockPreviousValue(false);
    angular_velocity->addTerm(new Triangle("NL", OUTPUT_ANG_NL_MIN, OUTPUT_ANG_NL_MED, OUTPUT_ANG_NL_MAX));
    angular_velocity->addTerm(new Triangle("NM", OUTPUT_ANG_NM_MIN, OUTPUT_ANG_NM_MED, OUTPUT_ANG_NM_MAX));
    angular_velocity->addTerm(new Triangle("N",  OUTPUT_ANG_N_MIN, OUTPUT_ANG_N_MED,OUTPUT_ANG_N_MAX));
    angular_velocity->addTerm(new Triangle("NS", OUTPUT_ANG_NS_MIN, OUTPUT_ANG_NS_MED,OUTPUT_ANG_NS_MAX));
    angular_velocity->addTerm(new Triangle("ZN", OUTPUT_ANG_ZN_MIN, OUTPUT_ANG_ZN_MED,OUTPUT_ANG_ZN_MAX));
    angular_velocity->addTerm(new Triangle("Z",  OUTPUT_ANG_Z_MIN, OUTPUT_ANG_Z_MED,OUTPUT_ANG_Z_MAX)); //----------
    angular_velocity->addTerm(new Triangle("ZP", OUTPUT_ANG_ZP_MIN, OUTPUT_ANG_ZP_MED,OUTPUT_ANG_ZP_MAX));
    angular_velocity->addTerm(new Triangle("PS", OUTPUT_ANG_PS_MIN, OUTPUT_ANG_PS_MED, OUTPUT_ANG_PS_MAX));
    angular_velocity->addTerm(new Triangle("P",  OUTPUT_ANG_P_MIN, OUTPUT_ANG_P_MED, OUTPUT_ANG_P_MAX));
    angular_velocity->addTerm(new Triangle("PM", OUTPUT_ANG_PM_MIN, OUTPUT_ANG_PM_MED, OUTPUT_ANG_PM_MAX));
    angular_velocity->addTerm(new Triangle("PL", OUTPUT_ANG_PL_MIN, OUTPUT_ANG_PL_MED, OUTPUT_ANG_PL_MAX));
    engine->addOutputVariable(angular_velocity);

    RuleBlock* ruleBlock = new RuleBlock;
    ruleBlock->setName("");
    ruleBlock->setDescription("");
    ruleBlock->setEnabled(true);
    //ruleBlock->setConjunction(fl::null);
    //ruleBlock->setDisjunction(fl::null);
    ruleBlock->setEnabled(true);
    ruleBlock->setConjunction(new Minimum);
    ruleBlock->setDisjunction(new Maximum);
    ruleBlock->setImplication(new Minimum);
  //  ruleBlock->setImplication(new AlgebraicProduct);
    ruleBlock->setActivation(new General);
    ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is S and angular_velocity is NL", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is S and angular_velocity is NM", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is S and angular_velocity is N", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is M and angular_velocity is NS", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is ZN then linear_velocity is L and angular_velocity is ZN", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is VL and angular_velocity is Z", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is L and angular_velocity is ZP", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is M and angular_velocity is PS", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is S and angular_velocity is P", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is S and angular_velocity is PM", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is S and angular_velocity is PL", engine));
    engine->addRuleBlock(ruleBlock);

    scalar input_angle = -3.14;
    FL_LOG(Op::str(input_angle) << ",, " << Uao_gtg->range())
    for (int i = 0; i < 50; ++i){
        input_angle += 0.125;
        Uao_gtg->setValue(input_angle);
        engine->process();
        FL_LOG( "input = " << Op::str(input_angle) << " => " << "output = linear:" << Op::str(linear_velocity->getValue()) << ", angular:" << Op::str(angular_velocity->getValue()));
    }
}