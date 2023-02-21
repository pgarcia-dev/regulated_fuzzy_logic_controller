
#include "fl/Headers.h"

int main(int argc, char **argv)
{
    using namespace fl;

    Engine *engine = new Engine;
    engine->setName("SFLC");
    engine->setDescription("");

    InputVariable* Uao_gtg = new InputVariable;
    Uao_gtg->setName("Uao_gtg");
    Uao_gtg->setDescription("");
    Uao_gtg->setEnabled(true);
    Uao_gtg->setRange(-3.8,3.8); 
    Uao_gtg->setLockValueInRange(false);
    Uao_gtg->addTerm(new Triangle("NL", -3.15, -3.0, -2.8));
    Uao_gtg->addTerm(new Triangle("NM", -2.8, -1.9, -1.1));
    Uao_gtg->addTerm(new Triangle("N", -1.1, -0.9, -0.6));
    Uao_gtg->addTerm(new Triangle("NS", -0.6, -0.5, -0.4));
    Uao_gtg->addTerm(new Triangle("ZN", -0.4, -0.25, -0.1));
    Uao_gtg->addTerm(new Triangle("Z", -0.1, 0, 0.1)); //----------
    Uao_gtg->addTerm(new Triangle("ZP", 0.1, 0.25, 0.4));
    Uao_gtg->addTerm(new Triangle("PS", 0.4, 0.5, 0.6));
    Uao_gtg->addTerm(new Triangle("P", 0.6, 0.9, 1.1));
    Uao_gtg->addTerm(new Triangle("PM",1.1, 1.9, 2.8));
    Uao_gtg->addTerm(new Triangle("PL", 2.8, 3.0, 3.15));
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
    angular_velocity->addTerm(new Triangle("NL", -1, -0.8, -0.6));
    angular_velocity->addTerm(new Triangle("NM", -0.6, -0.45, -0.31));
    angular_velocity->addTerm(new Triangle("N", -0.31, -0.25, -0.19));
    angular_velocity->addTerm(new Triangle("NS", -0.19, -0.15, -0.10));
    angular_velocity->addTerm(new Triangle("ZN", -0.1, -0.05, -0.025));
    angular_velocity->addTerm(new Triangle("Z", -0.025, 0, 0.025)); //----------
    angular_velocity->addTerm(new Triangle("ZP", 0.025, 0.05, 0.1));
    angular_velocity->addTerm(new Triangle("PS", 0.1, 0.15, 0.19));
    angular_velocity->addTerm(new Triangle("P", 0.19, 0.25, 0.31));
    angular_velocity->addTerm(new Triangle("PM",0.31, 0.45, 0.6));
    angular_velocity->addTerm(new Triangle("PL", 0.6, 0.8, 1));
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