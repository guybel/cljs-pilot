"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
require("should");
var index_1 = require("../index");
describe("encodeNmeaPacket", function () {
    it("throws on undefined message", function () {
        (function () {
            index_1.encodeNmeaPacket(undefined);
        }).should.throw("Packet must be given.");
    });
    it("throws on unknown type", function () {
        (function () {
            index_1.encodeNmeaPacket({ sentenceId: "foo" }, "II");
        }).should.throw("No known encoder for sentence ID \"foo\"");
    });
});
describe("parseNmeaSentence", function () {
    it("throws on bad checksum", function () {
        (function () {
            index_1.parseNmeaSentence("$IIHDM,201.5,M*21");
        }).should.throw("Invalid sentence: \"$IIHDM,201.5,M*21\".");
    });
    it("throws on unknown type", function () {
        (function () {
            index_1.parseNmeaSentence("$GPODD,123,456,Y*2A");
        }).should.throw("No known parser for sentence ID \"ODD\".");
    });
});
