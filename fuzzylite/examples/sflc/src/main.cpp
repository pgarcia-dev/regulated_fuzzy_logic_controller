
#include "fl/Headers.h"

int main(int argc, char **argv)
{
    using namespace fl;

    Engine *engine = new Engine;
    engine->setName("SFLC");
    engine->setDescription("");


    InputVariable* Ambient = new InputVariable;
    Ambient->setName("Ambient");
    Ambient->setDescription("");
    Ambient->setEnabled(true);
    Ambient->setRange(0.000, 1.000);
    Ambient->setLockValueInRange(false);
    Ambient->addTerm(new Triangle("DARK", 0.000, 0.150, 0.250));
    Ambient->addTerm(new Triangle("MEDIUM", 0.250, 0.500, 0.750));
    Ambient->addTerm(new Triangle("BRIGHT", 0.7500, 0.950, 1.000));
    engine->addInputVariable(Ambient);

    OutputVariable* Power = new OutputVariable;
    Power->setName("Power");
    Power->setDescription("");
    Power->setEnabled(true);
    Power->setRange(0.000, 1.000);
    Power->setLockValueInRange(false);
    Power->setAggregation(new Maximum);
    Power->setDefuzzifier(new Centroid(200));
    Power->setDefaultValue(fl::nan);
    Power->setLockPreviousValue(false);
    Power->addTerm(new Triangle("LOW", 0.000, 0.150, 0.2500));
    Power->addTerm(new Triangle("MEDIUM", 0.250, 0.500, 0.750));
    Power->addTerm(new Triangle("HIGH", 0.7500, 0.850, 1.000));
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

    scalar location = 0.1;
    Ambient->setValue(location);
    engine->process();

    FL_LOG( "input = " << Op::str(location) << " => " << "output = " << Op::str(Power->getValue()) );
    
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
    angular_velocity->addTerm(new Triangle("S2", 5, 5.5, 6));
    angular_velocity->addTerm(new Triangle("M2", 6, 6.5, 7));
    angular_velocity->addTerm(new Triangle("L2", 7, 7.5, 8));
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
    linear_velocity->addTerm(new Triangle("S3", 9, 9.5, 10));
    linear_velocity->addTerm(new Triangle("M3", 10, 10.5, 11));
    linear_velocity->addTerm(new Triangle("L3", 11, 11.5, 12));
    engine->addOutputVariable(linear_velocity);

    RuleBlock *mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if Uao_gtg is S then angular_velocity is S2 and linear_velocity is S3", engine));
    mamdani->addRule(Rule::parse("if Uao_gtg is M then angular_velocity is M2 and linear_velocity is M3", engine));
    mamdani->addRule(Rule::parse("if Uao_gtg is L then angular_velocity is L2 and linear_velocity is L3", engine));
    engine->addRuleBlock(mamdani);

    scalar location = 3.15;
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