
#include "fl/Headers.h"

int main(int argc, char **argv)
{
    using namespace fl;

    Engine *engine = new Engine;
    engine->setName("SFLC");
    engine->setDescription("");
/**
    InputVariable* Ambient = new InputVariable;
    Ambient->setName("Ambient");
    Ambient->setDescription("");
    Ambient->setEnabled(true);
    Ambient->setRange(0.000, 1.000);
    Ambient->setLockValueInRange(false);
    Ambient->addTerm(new Triangle("DARK", 0.000, 0.150, 0.250));
    Ambient->addTerm(new Triangle("MEDIUM", 0.250, 0.500, 0.750));
    Ambient->addTerm(new Triangle("BRIGHT", 0.750, 0.850, 1.000));
    engine->addInputVariable(Ambient);

    OutputVariable* Power = new OutputVariable;
    Power->setName("Power");
    Power->setDescription("");
    Power->setEnabled(true);
    Power->setRange(0.000, 1.000);
    Power->setLockValueInRange(false);
    Power->setAggregation(new Maximum);
    Power->setDefuzzifier(new LargestOfMaximum);
    Power->setDefaultValue(fl::nan);
    Power->setLockPreviousValue(false);
    Power->addTerm(new Triangle("LOW", 0.000, 0.150, 0.250));
    Power->addTerm(new Triangle("MEDIUM", 0.250, 0.500, 0.750));
    Power->addTerm(new Triangle("HIGH", 0.750, 0.750, 1.000));
    engine->addOutputVariable(Power);

    RuleBlock* ruleBlock = new RuleBlock;
    ruleBlock->setName("");
    ruleBlock->setDescription("");
    ruleBlock->setEnabled(true);
    ruleBlock->setConjunction(fl::null);
    ruleBlock->setDisjunction(fl::null);
    ruleBlock->setImplication(new Minimum);
    ruleBlock->setActivation(new General);
    ruleBlock->addRule(Rule::parse("if Ambient is DARK then Power is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Ambient is MEDIUM then Power is MEDIUM", engine));
    ruleBlock->addRule(Rule::parse("if Ambient is BRIGHT then Power is LOW", engine));
    engine->addRuleBlock(ruleBlock);


    scalar location = 0.001;
    Ambient->setValue(location);
    engine->process();

    FL_LOG( "inputt = " << Op::str(location) << " => " << "output = " << Op::str(Power->getValue()) );
**/


    InputVariable* Uao_gtg = new InputVariable;
    Uao_gtg->setName("Uao_gtg");
    Uao_gtg->setDescription("");
    Uao_gtg->setEnabled(true);
   // Uao_gtg->setRange(-3.8,3.8); ///////////////////////////////
    Uao_gtg->setRange(-10,10);
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
    linear_velocity->setRange(0.000, 1.000);
    linear_velocity->setLockValueInRange(false);
    linear_velocity->setAggregation(new Maximum);
    linear_velocity->setDefuzzifier(new LargestOfMaximum);
    linear_velocity->setDefaultValue(fl::nan);
    linear_velocity->setLockPreviousValue(false);
    linear_velocity->addTerm(new Triangle("LOW", 0.000, 0.150, 0.2500));
    linear_velocity->addTerm(new Triangle("MEDIUM", 0.250, 0.500, 0.750));
    linear_velocity->addTerm(new Triangle("HIGH", 0.7500, 0.850, 1.000));
    engine->addOutputVariable(linear_velocity);

    OutputVariable* angular_velocity = new OutputVariable;
    angular_velocity->setName("angular_velocity");
    angular_velocity->setDescription("");
    angular_velocity->setEnabled(true);
    angular_velocity->setRange(0.000, 1.000);
    angular_velocity->setLockValueInRange(false);
    angular_velocity->setAggregation(new Maximum);
    angular_velocity->setDefuzzifier(new LargestOfMaximum);
  //  angular_velocity->setDefuzzifier(new Centroid(200)); 


    angular_velocity->setDefaultValue(fl::nan);
    angular_velocity->setLockPreviousValue(false);
    angular_velocity->addTerm(new Triangle("LOW", 0.000, 0.150, 0.2500));
    angular_velocity->addTerm(new Triangle("MEDIUM", 0.250, 0.500, 0.750));
    angular_velocity->addTerm(new Triangle("HIGH", 0.7500, 0.850, 1.000));
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
    ruleBlock->addRule(Rule::parse("if Uao_gtg is NL then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is NM then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is N then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is NS then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is ZN then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is Z then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is ZP then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is PS then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is P then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is PM then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    ruleBlock->addRule(Rule::parse("if Uao_gtg is PL then linear_velocity is HIGH and angular_velocity is HIGH", engine));



    //ruleBlock->addRule(Rule::parse("if Uao_gtg is DARK then linear_velocity is HIGH and angular_velocity is HIGH", engine));
    //ruleBlock->addRule(Rule::parse("if Uao_gtg is MEDIUM then linear_velocity is MEDIUM", engine));
    //ruleBlock->addRule(Rule::parse("if Uao_gtg is BRIGHT then linear_velocity is LOW", engine));
    engine->addRuleBlock(ruleBlock);


    scalar input_angle = -3.14;
    FL_LOG(Op::str(input_angle) << ",, " << Uao_gtg->range())
    for (int i = 0; i < 50; ++i){
        input_angle += 0.125;
        Uao_gtg->setValue(input_angle);
        engine->process();
        FL_LOG( "input = " << Op::str(input_angle) << " => " << "output = " << Op::str(linear_velocity->getValue()) << ", " << Op::str(angular_velocity->getValue()));

    }

    //scalar location = -3.79;
    //Uao_gtg->setValue(location);
    //engine->process();

   // FL_LOG( "input = " << Op::str(location) << " => " << "output = " << Op::str(linear_velocity->getValue()) << ", " << Op::str(angular_velocity->getValue()));





    
   /** 
    InputVariable *Uao_gtg = new InputVariable;
    Uao_gtg->setName("Uao_gtg");
    Uao_gtg->setDescription("");
    Uao_gtg->setEnabled(true);
    Uao_gtg->setRange(1, 4);
    Uao_gtg->setLockValueInRange(false);
    Uao_gtg->addTerm(new Triangle("S", 1, 1.5, 2));
    Uao_gtg->addTerm(new Triangle("M", 2, 2.5, 3));
    Uao_gtg->addTerm(new Triangle("L", 3, 3.5, 4));
    // service->addTerm(new Trapezoid("excellent", 5.000, 7.500, 10.000, 10.000));
    engine->addInputVariable(Uao_gtg);

    OutputVariable *angular_velocity = new OutputVariable;
    angular_velocity->setName("angular_velocity");
    angular_velocity->setDescription("");
    angular_velocity->setEnabled(true);
    angular_velocity->setRange(5,8);
    angular_velocity->setLockValueInRange(false);
    angular_velocity->setAggregation(new Maximum);
    angular_velocity->setDefuzzifier(new Centroid(200)); //setDefuzzifier(new WeightedAverage("Automatic"));
    angular_velocity->setDefaultValue(fl::nan);
    angular_velocity->setLockPreviousValue(false);
    angular_velocity->addTerm(new Triangle("S2", 5.000, 5.500, 6.000));
    angular_velocity->addTerm(new Triangle("M2", 6.000, 6.500, 7.000));
    angular_velocity->addTerm(new Triangle("L2", 7.000, 7.5000, 8.000));
    engine->addOutputVariable(angular_velocity);

    OutputVariable *linear_velocity = new OutputVariable;
    linear_velocity->setName("linear_velocity");
    linear_velocity->setDescription("");
    linear_velocity->setEnabled(true);
    linear_velocity->setRange(9,12);
    linear_velocity->setLockValueInRange(false);
    linear_velocity->setAggregation(new Maximum);
    linear_velocity->setDefuzzifier(new Centroid(200)); //setDefuzzifier(new WeightedAverage("Automatic"));
    linear_velocity->setDefaultValue(fl::nan);
    linear_velocity->setLockPreviousValue(false);
    linear_velocity->addTerm(new Triangle("S3", 9.000, 9.500, 10.000));
    linear_velocity->addTerm(new Triangle("M3", 10.000, 10.500, 11.000));
    linear_velocity->addTerm(new Triangle("L3", 11.000, 11.500, 12.000));
    engine->addOutputVariable(linear_velocity);

    RuleBlock *mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new Minimum);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if Uao_gtg is S then angular_velocity is S2 and linear_velocity is S3", engine));
    mamdani->addRule(Rule::parse("if Uao_gtg is M then angular_velocity is M2 and linear_velocity is M3", engine));
    mamdani->addRule(Rule::parse("if Uao_gtg is L then angular_velocity is L2 and linear_velocity is L3", engine));
    engine->addRuleBlock(mamdani);

    scalar location = 1.1;
    Uao_gtg->setValue(location);
    engine->process();

    FL_LOG( "input = " << Op::str(location) << " => " << "output = " << Op::str(angular_velocity->getValue()) << ", " << Op::str(linear_velocity->getValue()));
**/







}
/**

    Engine* engine = new Engine;
    engine->setName("ObstacleAvoidance");
    engine->setDescription("");

    InputVariable* obstacle = new InputVariable;
    obstacle->setName("obstacle");
    obstacle->setDescription("");
    obstacle->setEnabled(true);
    obstacle->setRange(0.000, 1.000);
    obstacle->setLockValueInRange(false);
    obstacle->addTerm(new Ramp("left", 1.000, 0.000));
    obstacle->addTerm(new Ramp("right", 0.000, 1.000));
    engine->addInputVariable(obstacle);

    OutputVariable* mSteer = new OutputVariable;
    mSteer->setName("mSteer");
    mSteer->setDescription("");
    mSteer->setEnabled(true);
    mSteer->setRange(0.000, 1.000);
    mSteer->setLockValueInRange(false);
    mSteer->setAggregation(new Maximum);
    mSteer->setDefuzzifier(new Centroid(100));
    mSteer->setDefaultValue(fl::nan);
    mSteer->setLockPreviousValue(false);
    mSteer->addTerm(new Ramp("left", 1.000, 0.000));
    mSteer->addTerm(new Ramp("right", 0.000, 1.000));
    engine->addOutputVariable(mSteer);

    RuleBlock* mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if obstacle is left then mSteer is right", engine));
    mamdani->addRule(Rule::parse("if obstacle is right then mSteer is left", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    for (int i = 0; i <= 50; ++i){
        scalar location = obstacle->getMinimum() + i * (obstacle->range() / 50);
        obstacle->setValue(location);
        engine->process();
        FL_LOG("obstacle.input = " << Op::str(location) <<
            " => " << "steer.output = " << Op::str(steer->getValue()));
    }
}
**/