
#include "fl/Headers.h"

int main(int argc, char **argv)
{
    using namespace fl;

    Engine *engine = new Engine;
    engine->setName("SFLC");
    engine->setDescription("");

    InputVariable *Uao_gtg = new InputVariable;
    Uao_gtg->setName("Uao_gtg");
    Uao_gtg->setDescription("");
    Uao_gtg->setEnabled(true);
    // Uao_gtg->setRange(0.000, 1.000);
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
    //angular_velocity->setAggregation(new Maximum);
    angular_velocity->setDefuzzifier(new WeightedAverage("Automatic"));
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
    //linear_velocity->setAggregation(new Maximum);
    linear_velocity->setDefuzzifier(new WeightedAverage("Automatic"));
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
    mamdani->addRule(Rule::parse("if Uao_gtg is S then angular_velocity is S2", engine));
    mamdani->addRule(Rule::parse("if Uao_gtg is M then angular_velocity is M2", engine));
    mamdani->addRule(Rule::parse("if Uao_gtg is L then angular_velocity is L2", engine));
    engine->addRuleBlock(mamdani);



    scalar location = 1.5;
    Uao_gtg->setValue(location);
    engine->process();


    FL_LOG( "input = " << Op::str(location) << " => " << "output = " << Op::str(angular_velocity->getValue()) );

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