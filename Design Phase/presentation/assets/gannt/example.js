var tasks = [
{"startDate":new Date("Tue Feb 09 00:00:01 EST 2016"),"endDate":new Date("Tue Feb 23 00:00:01 EST 2016"),"taskName":"Design presentation","status":"SUCCEEDED"},
{"startDate":new Date("Tue Feb 23 00:00:01 EST 2016"),"endDate":new Date("Tue Mar 8 00:00:01 EST 2016"),"taskName":"Low level control module","status":"RUNNING"},
{"startDate":new Date("Tue Mar 08 00:00:01 EST 2016"),"endDate":new Date("Tue Mar 15 00:00:01 EST 2016"),"taskName":"High level control module","status":"RUNNING"},
{"startDate":new Date("Tue Mar 15 00:00:01 EST 2016"),"endDate":new Date("Tue Apr 05 00:00:01 EST 2016"),"taskName":"Object Detection module","status":"RUNNING"},
{"startDate":new Date("Tue Apr 05 00:00:01 EST 2016"),"endDate":new Date("Tue Apr 19 23:59:59 EST 2016"),"taskName":"Mission control","status":"RUNNING"},
{"startDate":new Date("Tue Apr 19 00:00:01 EST 2016"),"endDate":new Date("Tue Apr 26 23:59:59 EST 2016"),"taskName":"First Version (alpha)","status":"RUNNING"},
{"startDate":new Date("Tue Apr 26 00:00:01 EST 2016"),"endDate":new Date("Tue May 03 23:59:59 EST 2016"),"taskName":"Fully tested final version","status":"RUNNING"}
];

var taskStatus = {
    "SUCCEEDED" : "bar",
    "FAILED" : "bar-failed",
    "RUNNING" : "bar-running",
    "KILLED" : "bar-killed"
};
var taskNames = ["Fully tested final version", "First Version (alpha)", "Mission control", "Object Detection module", "High level control module", "Low level control module", "Design presentation"];
taskNames.reverse()
tasks.sort(function(a, b) {
    return a.endDate - b.endDate;
});
var maxDate = tasks[tasks.length - 1].endDate;
tasks.sort(function(a, b) {
    return a.startDate - b.startDate;
});
var minDate = tasks[0].startDate;

var format = "%d/%m/%y";

var gantt = d3.gantt().taskTypes(taskNames).taskStatus(taskStatus).tickFormat(format);
gantt(tasks);
